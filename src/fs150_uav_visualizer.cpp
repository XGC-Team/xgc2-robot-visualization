#include "xgc2_robot_visualization/fs150_uav_visualizer.hpp"

#include <algorithm>
#include <cmath>
#include <regex>
#include <string>

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>

namespace xgc2_robot_visualization {
namespace {

constexpr const char* kFs150BodyMesh = "package://fs150_description/meshes/iris.stl";
constexpr const char* kFs150PropCcwMesh = "package://fs150_description/meshes/iris_prop_ccw.dae";
constexpr const char* kFs150PropCwMesh = "package://fs150_description/meshes/iris_prop_cw.dae";

struct RotorVisual {
    const char* name;
    geometry_msgs::Vector3 offset;
    const char* mesh;
    double direction;
};

geometry_msgs::Vector3 makeVector3(double x, double y, double z) {
    geometry_msgs::Vector3 out;
    out.x = x;
    out.y = y;
    out.z = z;
    return out;
}

geometry_msgs::Point makePoint(double x, double y, double z) {
    geometry_msgs::Point out;
    out.x = x;
    out.y = y;
    out.z = z;
    return out;
}

geometry_msgs::Quaternion makeQuaternion(double x, double y, double z, double w) {
    geometry_msgs::Quaternion out;
    out.x = x;
    out.y = y;
    out.z = z;
    out.w = w;
    return out;
}

std_msgs::ColorRGBA makeColor(double r, double g, double b, double a) {
    std_msgs::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;
    return color;
}

const std::vector<RotorVisual>& fs150Rotors() {
    static const std::vector<RotorVisual> rotors = {
        {"rotor_0", makeVector3(0.13, -0.22, 0.023), kFs150PropCcwMesh, 1.0},
        {"rotor_1", makeVector3(-0.13, 0.20, 0.023), kFs150PropCcwMesh, 1.0},
        {"rotor_2", makeVector3(0.13, 0.22, 0.023), kFs150PropCwMesh, -1.0},
        {"rotor_3", makeVector3(-0.13, -0.20, 0.023), kFs150PropCwMesh, -1.0},
    };
    return rotors;
}

geometry_msgs::Quaternion normalize(const geometry_msgs::Quaternion& q) {
    const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    if (!std::isfinite(norm) || norm < 1.0e-9) {
        return makeQuaternion(0.0, 0.0, 0.0, 1.0);
    }
    return makeQuaternion(q.x / norm, q.y / norm, q.z / norm, q.w / norm);
}

geometry_msgs::Quaternion multiply(const geometry_msgs::Quaternion& lhs, const geometry_msgs::Quaternion& rhs) {
    const geometry_msgs::Quaternion a = normalize(lhs);
    const geometry_msgs::Quaternion b = normalize(rhs);
    return makeQuaternion(a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
                          a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
                          a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
                          a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z);
}

geometry_msgs::Quaternion multiplyRaw(const geometry_msgs::Quaternion& a, const geometry_msgs::Quaternion& b) {
    return makeQuaternion(a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
                          a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
                          a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
                          a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z);
}

geometry_msgs::Quaternion yawQuaternion(double yaw) {
    return makeQuaternion(0.0, 0.0, std::sin(0.5 * yaw), std::cos(0.5 * yaw));
}

geometry_msgs::Vector3 rotateVector(const geometry_msgs::Quaternion& q, const geometry_msgs::Vector3& v) {
    const geometry_msgs::Quaternion qn = normalize(q);
    const geometry_msgs::Quaternion vq = makeQuaternion(v.x, v.y, v.z, 0.0);
    const geometry_msgs::Quaternion qi = makeQuaternion(-qn.x, -qn.y, -qn.z, qn.w);
    const geometry_msgs::Quaternion out = multiplyRaw(multiplyRaw(qn, vq), qi);
    return makeVector3(out.x, out.y, out.z);
}

visualization_msgs::Marker makeMeshMarker(const std::string& ns, int id, const std::string& mesh,
                                          const std::string& frame_id, const geometry_msgs::Pose& pose,
                                          const ros::Time& stamp, const std_msgs::ColorRGBA& color, double scale) {
    visualization_msgs::Marker marker;
    marker.header.stamp = stamp;
    marker.header.frame_id = frame_id;
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.mesh_resource = mesh;
    marker.pose = pose;
    marker.scale.x = scale;
    marker.scale.y = scale;
    marker.scale.z = scale;
    marker.color = color;
    return marker;
}

geometry_msgs::TransformStamped makeTransform(const std::string& parent_frame, const std::string& child_frame,
                                              const geometry_msgs::Pose& pose, const ros::Time& stamp) {
    geometry_msgs::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = parent_frame;
    transform.child_frame_id = child_frame;
    transform.transform.translation = makeVector3(pose.position.x, pose.position.y, pose.position.z);
    transform.transform.rotation = pose.orientation;
    return transform;
}

std::string trailingNumber(const std::string& name) {
    static const std::regex pattern("([0-9]+)$");
    std::smatch match;
    return std::regex_search(name, match, pattern) ? match.str(1) : std::string();
}

std::string displayName(const UavVisualState& state) {
    const std::string number = trailingNumber(state.name);
    return "UAV " + (number.empty() ? state.name : number);
}

} // namespace

Fs150UavVisualizer::Fs150UavVisualizer(const Config& config) : config_(config) {
    config_.path_publish_rate = std::max(1.0, config_.path_publish_rate);
    config_.path_limit = std::max(2, config_.path_limit);
    config_.mesh_scale = std::max(0.001, config_.mesh_scale);
    config_.rotor_speed_rad_s = std::max(0.0, config_.rotor_speed_rad_s);
}

void Fs150UavVisualizer::append(const UavVisualState& state, visualization_msgs::MarkerArray* markers,
                                std::vector<geometry_msgs::TransformStamped>* transforms) {
    ModelVisualState& visual = models_[state.name];
    if (visual.rotor_phases.size() != fs150Rotors().size()) {
        visual.rotor_phases.assign(fs150Rotors().size(), 0.0);
    }

    updateRotorPhases(&visual, state);
    updatePath(&visual, state);

    transforms->push_back(makeTransform(config_.frame_id, state.name + "/base_link", state.pose, state.stamp));
    addBodyMarker(state, markers);
    addRotorMarkers(state, visual, markers, transforms);
    addPathMarker(state, visual, markers);
    addLabelMarker(state, markers);
}

void Fs150UavVisualizer::updateRotorPhases(ModelVisualState* visual, const UavVisualState& state) const {
    const double dt =
        visual->last_update_stamp.isZero() ? 0.0 : std::max(0.0, (state.stamp - visual->last_update_stamp).toSec());
    const double rotor_speed =
        state.rotor_speed_rad_s > 0.0 ? state.rotor_speed_rad_s : config_.rotor_speed_rad_s;
    if (state.rotors_active && dt > 0.0) {
        const std::vector<RotorVisual>& rotors = fs150Rotors();
        for (std::size_t i = 0; i < rotors.size(); ++i) {
            visual->rotor_phases[i] =
                std::fmod(visual->rotor_phases[i] + rotors[i].direction * rotor_speed * dt,
                          2.0 * M_PI);
        }
    }
    visual->last_update_stamp = state.stamp;
}

void Fs150UavVisualizer::updatePath(ModelVisualState* visual, const UavVisualState& state) const {
    if (!visual->last_path_stamp.isZero() &&
        (state.stamp - visual->last_path_stamp).toSec() < 1.0 / config_.path_publish_rate) {
        return;
    }
    if (static_cast<int>(visual->path.size()) >= config_.path_limit) {
        visual->path.pop_front();
    }
    visual->path.push_back(state.pose.position);
    visual->last_path_stamp = state.stamp;
}

void Fs150UavVisualizer::addBodyMarker(const UavVisualState& state, visualization_msgs::MarkerArray* markers) const {
    markers->markers.push_back(makeMeshMarker(state.name + "_body", 0, kFs150BodyMesh, config_.frame_id, state.pose,
                                              state.stamp, makeColor(0.84, 0.71, 0.10, 1.0), config_.mesh_scale));
}

void Fs150UavVisualizer::addRotorMarkers(const UavVisualState& state, const ModelVisualState& visual,
                                         visualization_msgs::MarkerArray* markers,
                                         std::vector<geometry_msgs::TransformStamped>* transforms) const {
    const std::vector<RotorVisual>& rotors = fs150Rotors();
    for (std::size_t i = 0; i < rotors.size(); ++i) {
        const RotorVisual& rotor = rotors[i];
        const double phase = visual.rotor_phases[i];

        geometry_msgs::Pose rotor_relative_pose;
        rotor_relative_pose.position = makePoint(rotor.offset.x, rotor.offset.y, rotor.offset.z);
        rotor_relative_pose.orientation = yawQuaternion(phase);
        transforms->push_back(
            makeTransform(state.name + "/base_link", state.name + "/" + rotor.name, rotor_relative_pose, state.stamp));

        geometry_msgs::Pose rotor_pose;
        const geometry_msgs::Vector3 offset = rotateVector(state.pose.orientation, rotor.offset);
        rotor_pose.position = makePoint(state.pose.position.x + offset.x, state.pose.position.y + offset.y,
                                        state.pose.position.z + offset.z);
        rotor_pose.orientation = multiply(state.pose.orientation, yawQuaternion(phase));

        const bool blue_rotor = std::string(rotor.name) == "rotor_0" || std::string(rotor.name) == "rotor_2";
        const std_msgs::ColorRGBA rotor_color =
            blue_rotor ? makeColor(0.10, 0.20, 0.90, 1.0) : makeColor(0.12, 0.12, 0.12, 1.0);
        markers->markers.push_back(makeMeshMarker(state.name + "_" + rotor.name, static_cast<int>(i) + 1, rotor.mesh,
                                                  config_.frame_id, rotor_pose, state.stamp, rotor_color,
                                                  config_.mesh_scale));
    }
}

void Fs150UavVisualizer::addPathMarker(const UavVisualState& state, const ModelVisualState& visual,
                                       visualization_msgs::MarkerArray* markers) const {
    if (visual.path.size() < 2) {
        return;
    }

    visualization_msgs::Marker marker;
    marker.header.stamp = state.stamp;
    marker.header.frame_id = config_.frame_id;
    marker.ns = state.name + "_actual_path";
    marker.id = 10;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.points.assign(visual.path.begin(), visual.path.end());
    marker.scale.x = 0.018;
    marker.color.r = 0.0;
    marker.color.g = 0.55;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
    markers->markers.push_back(marker);
}

void Fs150UavVisualizer::addLabelMarker(const UavVisualState& state, visualization_msgs::MarkerArray* markers) const {
    visualization_msgs::Marker marker;
    marker.header.stamp = state.stamp;
    marker.header.frame_id = config_.frame_id;
    marker.ns = state.name + "_label";
    marker.id = 11;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position =
        makePoint(state.pose.position.x, state.pose.position.y, state.pose.position.z + 0.55);
    marker.pose.orientation = makeQuaternion(0.0, 0.0, 0.0, 1.0);
    marker.scale.z = 0.32;
    marker.color = makeColor(1.0, 1.0, 1.0, 1.0);
    marker.text = displayName(state);
    markers->markers.push_back(marker);
}

} // namespace xgc2_robot_visualization
