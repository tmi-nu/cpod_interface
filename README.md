# cpod_interface
Autoware.universe ROS2 C+Pod Interface 

# coms_connector
## How to launch
```
ros2 launch coms_connector plc_connector.launch.py 
```

## Nodes
* `plc_sender` : This node subscribes `/plc_control_packet` and sends control commands to plc.
* `plc_receiver` : This node receives sensor data from plc and publish to `plc_sensor_packet`.

## Topics
| name | message type |
|---|---|
|/plc_control_packet | coms_msgs::msg::ComsControlPacket
|/plc_sensor_packet | coms_msgs::msg::ComsSensorPacket


## Parameters
See `src/coms_connector/config/plc_connector.param.yaml`
* destination : ip address 
* port : port number

# coms_converter
## How to launch
```
ros2 launch coms_converter plc_converter.launch.py
```
## Node
`/plc_converter`

## Topics
### Subscriber
| name | message type |
|---|---|
|/plc_sensor_packet |coms_msgs::msg::ComsSensorPacket|
|/twist_cmd| geometry_msgs::msg::TwistStamped |
|/curr_twist | geometry_msgs::msg::TwistStamped |
|/accel_cmd | autoware_msgs::msg::AccelCmd |
|/steer_cmd | autoware_msgs::msg::SteerCmd |
|/brake_cmd | autoware_msgs::msg::BrakeCmd |
|/lamp_cmd|autoware_msgs::msg::LampCmd|
|/indicator_cmd|autoware_msgs::msg::IndicatorCmd|
|/gear_cmd|tablet_socket_msgs::msg::GearCmd|
|/mode_cmd|tablet_socket_msgs::msg::ModeCmd|
|/estimate_twist|geometry_msgs::msg::TwistStamped|

### Publisher
| name | message type |
|---|---|
|/odom_twist | geometry_msgs::msg::TwistStamped|
|/curr_pose | geometry_msgs::msg::PoseStamped |
|/plc_sensor_packet | coms_msgs::msg::ComsControlPacket|



## Parameters
See `src/coms_converter/config/plc_connector.param.yaml`
| var| default | description|
|----|---------|----| 
|`direct_control_flag`|false| Use direct control|
|`use_low_pass_filter`|true | Use low pass filter for sensor data|
|`use_median_filter`|true| Use median filter for sensor data|
|`lowpass_filter_cutoff_frequency_wr`| 50 | The cut off frequency of low pass filter used for the velocity of the right wheel|
|`lowpass_filter_cutoff_frequency_wl`| 50 | The cut off frequency of low pass filter used for the velocity of the left wheel|
|`lowpass_filter_sampling_frequency_wr`| 100 | The sampling frequency of low pass filter used for the velocity of the right wheel|
|`lowpass_filter_sampling_frequency_wl`| 100 | The sampling frequency of low pass filter used for the velocity of the left wheel|
|`lowpass_filter_size_wr`| 20 | The size of low pass filter used for the right wheel|
|`lowpass_filter_size_wl`| 20 | The size of low pass filter used for the left wheel|
|`median_filter_size_wr`|5| The size of median filter used for the right wheel|
|`median_filter_size_wl`|5| The size of median filter used for the left wheel|
|`tire_radius`|1| Radius of the wheel|
|`tire_distance`|1| Distance between the right wheel and left one|
|`encoder_pulse_resolution`|1| Number of pulse during wheel rotates $2\pi$|