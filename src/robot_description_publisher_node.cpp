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
// It sets the parameters and stops. There is nothing to poll: a ROS parameter
// is retained by the master for as long as the master lives, and the roster is
// frozen for the run.

#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <fstream>
#include <ros/ros.h>

namespace {

// Robot names arrive as a comma separated list, matching how the rest of the
// XGC2 process catalog passes fleet composition into a ROS node.
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

// A namespace is the robot's own name. Keeping the two identical is what lets a
// viewer derive the parameter name and the transform prefix from one fact.
std::string descriptionParameter(const std::string& robot_name) {
    return "/" + robot_name + "/robot_description";
}

}  // namespace

int main(int argc, char** argv) {
    ros::init(argc, argv, "xgc2_robot_description_publisher");
    ros::NodeHandle node;
    ros::NodeHandle private_node("~");

    // One description file per robot kind, and one robot roster per kind. A kind
    // with no robots contributes nothing, so an Experiment that flies only
    // multirotors does not need a Scout description on disk.
    std::map<std::string, std::string> roster_parameter;
    roster_parameter["fs150"] = "fs150_models";
    roster_parameter["scout"] = "scout_models";
    roster_parameter["mecanum"] = "mecanum_models";

    std::map<std::string, std::string> description_parameter;
    description_parameter["fs150"] = "fs150_description_path";
    description_parameter["scout"] = "scout_description_path";
    description_parameter["mecanum"] = "mecanum_description_path";

    bool published_any = false;
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
            // Fail closed on a name that cannot become a legal parameter path:
            // a viewer would silently render nothing for that robot.
            if (names[index].find('/') != std::string::npos) {
                ROS_FATAL_STREAM("robot name must not contain a slash: " << names[index]);
                return 1;
            }
            node.setParam(descriptionParameter(names[index]), description);
            published_any = true;
        }
        ROS_INFO_STREAM("published " << kind << " robot description to " << names.size()
                                     << " robot(s) from " << path << " (" << description.size()
                                     << " bytes)");
    }

    if (!published_any) {
        ROS_FATAL("no robots were listed; refusing to start a description publisher with nothing"
                  " to publish");
        return 1;
    }

    // Readiness for this node is the presence of the parameters it just set, so
    // it stays alive only to keep its ROS graph registration -- a supervisor
    // that sees the node vanish should treat the descriptions as unowned.
    ros::spin();
    return 0;
}
