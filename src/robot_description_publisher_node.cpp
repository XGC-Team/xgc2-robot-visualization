// Publishes one visual URDF per robot onto the parameter server so a viewer can
// load each robot's geometry once, by name, instead of receiving mesh
// references on a topic every frame.
//
// This node is deliberately source agnostic, like the rest of this package: it
// takes a robot roster and a per-kind description file and knows nothing about
// where a robot's pose comes from. A Gazebo session, a motion-capture rig or a
// physical fleet all end up with the same parameters, so the viewer layout that
// consumes them does not change between simulation and a real flight.
//
// Geometry is published to /<robot>/visual_robot_description — never
// /robot_description. Gazebo spawn overwrites the latter with control URDFs
// (Scout especially), which produces black meshes and broken joint trees in
// Lichtblick. Parameters are re-asserted periodically so late spawners cannot
// win a race against the viewer path.

#include <map>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <fstream>
#include <ros/ros.h>

namespace {

std::vector<std::string> splitNames(const std::string& value) {
    std::vector<std::string> names;
    std::stringstream stream(value);
    std::string item;
    while (std::getline(stream, item, ',')) {
        const std::size_t begin = item.find_first_not_of(" \t");
        if (begin == std::string::npos) {
            continue;
        }
        const std::size_t end = item.find_last_not_of(" \t");
        names.push_back(item.substr(begin, end - begin + 1));
    }
    return names;
}

bool readFile(const std::string& path, std::string* contents, std::string* error) {
    std::ifstream stream(path.c_str(), std::ios::in | std::ios::binary);
    if (!stream) {
        *error = "cannot open robot description file: " + path;
        return false;
    }
    std::ostringstream buffer;
    buffer << stream.rdbuf();
    if (!stream.good() && !stream.eof()) {
        *error = "cannot read robot description file: " + path;
        return false;
    }
    *contents = buffer.str();
    if (contents->empty()) {
        *error = "robot description file is empty: " + path;
        return false;
    }
    return true;
}

// Dedicated viewer parameter. Do not use /robot_description — simulators own it.
std::string descriptionParameter(const std::string& robot_name) {
    return "/" + robot_name + "/visual_robot_description";
}

void publishAll(ros::NodeHandle& node,
                const std::vector<std::pair<std::string, std::string> >& published) {
    for (std::size_t index = 0; index < published.size(); ++index) {
        node.setParam(published[index].first, published[index].second);
    }
}

}  // namespace

int main(int argc, char** argv) {
    ros::init(argc, argv, "xgc2_robot_description_publisher");
    ros::NodeHandle node;
    ros::NodeHandle private_node("~");

    std::map<std::string, std::string> roster_parameter;
    roster_parameter["fs150"] = "fs150_models";
    roster_parameter["scout"] = "scout_models";
    roster_parameter["mecanum"] = "mecanum_models";

    std::map<std::string, std::string> description_parameter;
    description_parameter["fs150"] = "fs150_description_path";
    description_parameter["scout"] = "scout_description_path";
    description_parameter["mecanum"] = "mecanum_description_path";

    // param name -> URDF text
    std::vector<std::pair<std::string, std::string> > published;

    for (std::map<std::string, std::string>::const_iterator it = roster_parameter.begin();
         it != roster_parameter.end(); ++it) {
        const std::string& kind = it->first;

        std::string roster;
        private_node.param<std::string>(it->second, roster, std::string());
        const std::vector<std::string> names = splitNames(roster);
        if (names.empty()) {
            continue;
        }

        std::string path;
        private_node.param<std::string>(description_parameter[kind], path, std::string());
        if (path.empty()) {
            ROS_FATAL_STREAM("robot kind " << kind << " has robots " << roster
                                           << " but no description path; refusing to publish a"
                                              " fleet the viewer cannot render");
            return 1;
        }

        std::string description;
        std::string error;
        if (!readFile(path, &description, &error)) {
            ROS_FATAL_STREAM(error);
            return 1;
        }

        for (std::size_t index = 0; index < names.size(); ++index) {
            if (names[index].find('/') != std::string::npos) {
                ROS_FATAL_STREAM("robot name must not contain a slash: " << names[index]);
                return 1;
            }
            published.push_back(std::make_pair(descriptionParameter(names[index]), description));
        }
        ROS_INFO_STREAM("prepared " << kind << " visual robot description for " << names.size()
                                    << " robot(s) from " << path << " (" << description.size()
                                    << " bytes) on */visual_robot_description");
    }

    if (published.empty()) {
        ROS_FATAL("no robots were listed; refusing to start a description publisher with nothing"
                  " to publish");
        return 1;
    }

    publishAll(node, published);
    ROS_INFO_STREAM("published " << published.size()
                                 << " visual_robot_description parameter(s); re-asserting at 0.5 Hz");

    // Re-assert so a late Gazebo/xacro load cannot leave the viewer on a stale
    // control URDF if something still writes the wrong parameter name, and so
    // parameter-subscribe clients see a fresh update after spawn storms.
    ros::Rate rate(0.5);
    while (ros::ok()) {
        publishAll(node, published);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
