source /opt/ros/galactic/setup.bash
ros2 topic pub /twist_cmd geometry_msgs/msg/TwistStamped "{twist:{linear: {x: 2.0}, angular:{z: 1.8}}}"