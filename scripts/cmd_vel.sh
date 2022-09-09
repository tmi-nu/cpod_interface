source /opt/ros/galactic/setup.bash
ros2 topic pub /twist_cmd geometry_msgs/msg/TwistStamped "{twist:{linear: {x: 0.5}, angular:{z: 0.0}}}"