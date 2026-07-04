# xgc2_robot_visualization

Reusable RViz marker visualizers for XGC2 robot assets.

This package is source agnostic: it accepts robot visual state structs and
produces markers plus TF transforms. Gazebo, MAVROS, VRPN, VIO, or real-robot
adapters should live outside this package.
