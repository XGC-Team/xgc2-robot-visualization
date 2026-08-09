// Publishes one visual URDF per frozen Experiment Robot and, when requested by
// that Robot's contribution, supervises a namespaced robot_state_publisher.
//
// The managed process receives one canonical JSON roster through
// XGC2_ROBOT_VISUALIZATION_ROSTER. It resolves package-relative URDF files via
// the ROS package index at runtime. No workstation path, deployment username,
// Robot kind switch, or singleton /robot_description participates.

#include <cerrno>
#include <csignal>
#include <cstdlib>
#include <fstream>
#include <set>
#include <sstream>
#include <string>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <utility>
#include <vector>

#include <ros/master.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>

#include "robot_description_runtime.hpp"

namespace {

constexpr const char* kRosterEnvironment = "XGC2_ROBOT_VISUALIZATION_ROSTER";

volatile std::sig_atomic_t shutdown_requested = 0;

using xgc2_robot_visualization::RobotDescription;

struct ChildProcess {
    pid_t pid;
    std::string robot_name;
    std::string node_name;
};

void requestShutdown(int) {
    shutdown_requested = 1;
}

bool readFile(const std::string& file, std::string* contents, std::string* error) {
    std::ifstream stream(file.c_str(), std::ios::in | std::ios::binary);
    if (!stream) {
        *error = "cannot open installed Robot description file: " + file;
        return false;
    }
    std::ostringstream buffer;
    buffer << stream.rdbuf();
    if (!stream.good() && !stream.eof()) {
        *error = "cannot read installed Robot description file: " + file;
        return false;
    }
    *contents = buffer.str();
    if (contents->empty()) {
        *error = "installed Robot description file is empty: " + file;
        return false;
    }
    return true;
}

std::string visualDescriptionParameter(const RobotDescription& robot) {
    return robot.ros_namespace + "/visual_robot_description";
}

std::string statePublisherDescriptionParameter(const RobotDescription& robot) {
    return robot.ros_namespace + "/robot_description";
}

void publishAll(ros::NodeHandle& node,
                const std::vector<std::pair<std::string, std::string> >& published) {
    for (const auto& parameter : published) {
        node.setParam(parameter.first, parameter.second);
    }
}

bool childExited(const ChildProcess& child, std::string* error) {
    int status = 0;
    const pid_t result = waitpid(child.pid, &status, WNOHANG);
    if (result == 0) {
        return false;
    }
    if (result < 0) {
        *error = "cannot inspect robot_state_publisher for " + child.robot_name +
                 ": errno=" + std::to_string(errno);
    } else if (WIFEXITED(status)) {
        *error = "robot_state_publisher for " + child.robot_name +
                 " exited with status " + std::to_string(WEXITSTATUS(status));
    } else if (WIFSIGNALED(status)) {
        *error = "robot_state_publisher for " + child.robot_name +
                 " exited on signal " + std::to_string(WTERMSIG(status));
    } else {
        *error = "robot_state_publisher for " + child.robot_name + " stopped unexpectedly";
    }
    return true;
}

bool startRobotStatePublisher(const RobotDescription& robot,
                              ChildProcess* child,
                              std::string* error) {
    const std::string package_path = ros::package::getPath("robot_state_publisher");
    const std::string package_suffix = "/share/robot_state_publisher";
    if (package_path.size() <= package_suffix.size() ||
        package_path.compare(package_path.size() - package_suffix.size(),
                             package_suffix.size(), package_suffix) != 0) {
        *error = "APT-installed ROS package robot_state_publisher is unavailable";
        return false;
    }
    const std::string executable =
        package_path.substr(0, package_path.size() - package_suffix.size()) +
        "/lib/robot_state_publisher/robot_state_publisher";
    if (access(executable.c_str(), X_OK) != 0) {
        *error = "APT-installed robot_state_publisher executable is unavailable: " + executable;
        return false;
    }
    const pid_t pid = fork();
    if (pid < 0) {
        *error = "cannot fork robot_state_publisher for " + robot.name +
                 ": errno=" + std::to_string(errno);
        return false;
    }
    if (pid == 0) {
        const std::string node_name = "__name:=robot_state_publisher";
        const std::string node_namespace = "__ns:=" + robot.ros_namespace;
        // Noetic reads the private ROS 1 parameter tf_prefix. frame_prefix is
        // the ROS 2 contract and silently leaves a disconnected, unprefixed
        // tree on Noetic.
        const std::string frame_prefix = "_tf_prefix:=" + robot.name;
        const std::string joint_remapping = "joint_states:=" + robot.joint_state_topic;
        // Execute the installed binary directly. rosrun may retain a wrapper
        // process, which would let the actual RSP survive when the supervised
        // child PID is terminated.
        execl(executable.c_str(), executable.c_str(), node_name.c_str(),
              node_namespace.c_str(), frame_prefix.c_str(), joint_remapping.c_str(),
              static_cast<char*>(nullptr));
        _exit(127);
    }
    child->pid = pid;
    child->robot_name = robot.name;
    child->node_name = robot.ros_namespace + "/robot_state_publisher";
    return true;
}

bool allChildNodesReady(const std::vector<ChildProcess>& children) {
    if (children.empty()) {
        return true;
    }
    ros::V_string nodes;
    if (!ros::master::getNodes(nodes)) {
        return false;
    }
    const std::set<std::string> active(nodes.begin(), nodes.end());
    for (const auto& child : children) {
        if (active.count(child.node_name) == 0) {
            return false;
        }
    }
    return true;
}

bool waitForChildren(const std::vector<ChildProcess>& children, std::string* error) {
    ros::WallRate rate(10.0);
    for (int attempt = 0; attempt < 200 && ros::ok() && !shutdown_requested; ++attempt) {
        for (const auto& child : children) {
            if (childExited(child, error)) {
                return false;
            }
        }
        if (allChildNodesReady(children)) {
            return true;
        }
        ros::spinOnce();
        rate.sleep();
    }
    *error = "timed out waiting for namespaced robot_state_publisher nodes";
    return false;
}

void stopChildren(const std::vector<ChildProcess>& children) {
    std::set<pid_t> remaining;
    for (const auto& child : children) {
        if (kill(child.pid, SIGTERM) == 0 || errno == EPERM) {
            remaining.insert(child.pid);
        }
    }
    for (int attempt = 0; attempt < 100 && !remaining.empty(); ++attempt) {
        for (auto it = remaining.begin(); it != remaining.end();) {
            int status = 0;
            const pid_t result = waitpid(*it, &status, WNOHANG);
            if (result == *it || (result < 0 && errno == ECHILD)) {
                it = remaining.erase(it);
            } else {
                ++it;
            }
        }
        if (!remaining.empty()) {
            usleep(50000);
        }
    }
    for (const pid_t pid : remaining) {
        kill(pid, SIGKILL);
        waitpid(pid, nullptr, 0);
    }
}

}  // namespace

int main(int argc, char** argv) {
    ros::init(argc, argv, "xgc2_robot_description_publisher",
              ros::init_options::NoSigintHandler);
    std::signal(SIGINT, requestShutdown);
    std::signal(SIGTERM, requestShutdown);

    const char* raw_roster = std::getenv(kRosterEnvironment);
    if (raw_roster == nullptr || std::string(raw_roster).empty()) {
        ROS_FATAL_STREAM(kRosterEnvironment << " is required");
        return 1;
    }
    std::vector<RobotDescription> robots;
    std::string error;
    if (!xgc2_robot_visualization::readRobotVisualizationRoster(
            raw_roster, &robots, &error)) {
        ROS_FATAL_STREAM(error);
        return 1;
    }

    ros::NodeHandle node;
    std::vector<std::pair<std::string, std::string> > published;
    std::vector<ChildProcess> children;
    for (const auto& robot : robots) {
        const std::string package_path = ros::package::getPath(robot.description_package);
        if (package_path.empty()) {
            ROS_FATAL_STREAM("APT-installed ROS package " << robot.description_package
                                                           << " is unavailable for " << robot.name);
            stopChildren(children);
            return 1;
        }
        const std::string description_file = package_path + "/" + robot.description_file;
        std::string description;
        if (!readFile(description_file, &description, &error)) {
            ROS_FATAL_STREAM(error);
            stopChildren(children);
            return 1;
        }
        published.emplace_back(visualDescriptionParameter(robot), description);
        if (robot.robot_state_publisher) {
            published.emplace_back(statePublisherDescriptionParameter(robot), description);
        }
        ROS_INFO_STREAM("prepared namespaced visual description for " << robot.name << " from "
                                                                        << robot.description_package
                                                                        << "/" << robot.description_file);
    }

    publishAll(node, published);
    for (const auto& robot : robots) {
        if (!robot.robot_state_publisher) {
            continue;
        }
        ChildProcess child;
        if (!startRobotStatePublisher(robot, &child, &error)) {
            ROS_FATAL_STREAM(error);
            stopChildren(children);
            return 1;
        }
        children.push_back(child);
    }
    if (!waitForChildren(children, &error)) {
        ROS_FATAL_STREAM(error);
        stopChildren(children);
        return 1;
    }

    ros::Publisher ready = node.advertise<std_msgs::Empty>(
        "/xgc/robot_descriptions/ready", 1, true);
    ready.publish(std_msgs::Empty());
    ROS_INFO_STREAM("published " << robots.size()
                                  << " frozen visual description(s) and supervised "
                                  << children.size() << " robot_state_publisher process(es)");

    ros::Rate rate(0.5);
    while (ros::ok() && !shutdown_requested) {
        publishAll(node, published);
        for (const auto& child : children) {
            if (childExited(child, &error)) {
                ROS_FATAL_STREAM(error);
                stopChildren(children);
                return 1;
            }
        }
        ros::spinOnce();
        rate.sleep();
    }
    stopChildren(children);
    return 0;
}
