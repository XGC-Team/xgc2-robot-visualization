#pragma once

#include <deque>
#include <map>
#include <string>
#include <vector>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/time.h>
#include <visualization_msgs/MarkerArray.h>

namespace xgc2_robot_visualization {

struct UavVisualState {
    std::string name;
    geometry_msgs::Pose pose;
    bool rotors_active{false};
    ros::Time stamp;
};

class Fs150UavVisualizer {
  public:
    struct Config {
        std::string frame_id{"world"};
        double rotor_speed_rad_s{70.0};
        double mesh_scale{1.0};
        double path_publish_rate{10.0};
        int path_limit{3000};
    };

    explicit Fs150UavVisualizer(const Config& config);

    void append(const UavVisualState& state, visualization_msgs::MarkerArray* markers,
                std::vector<geometry_msgs::TransformStamped>* transforms);

  private:
    struct ModelVisualState {
        ros::Time last_update_stamp;
        ros::Time last_path_stamp;
        std::deque<geometry_msgs::Point> path;
        std::vector<double> rotor_phases;
    };

    void updateRotorPhases(ModelVisualState* visual, const UavVisualState& state) const;
    void updatePath(ModelVisualState* visual, const UavVisualState& state) const;
    void addBodyMarker(const UavVisualState& state, visualization_msgs::MarkerArray* markers) const;
    void addRotorMarkers(const UavVisualState& state, const ModelVisualState& visual,
                         visualization_msgs::MarkerArray* markers,
                         std::vector<geometry_msgs::TransformStamped>* transforms) const;
    void addPathMarker(const UavVisualState& state, const ModelVisualState& visual,
                       visualization_msgs::MarkerArray* markers) const;
    void addLabelMarker(const UavVisualState& state, visualization_msgs::MarkerArray* markers) const;

    Config config_;
    std::map<std::string, ModelVisualState> models_;
};

} // namespace xgc2_robot_visualization
