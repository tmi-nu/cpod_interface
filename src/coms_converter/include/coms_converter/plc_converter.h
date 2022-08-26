#ifndef PLC_CONVERTER_H_
#define PLC_CONVERTER_H_

// #include "ros/ros.h"
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/string.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <memory>
#include <functional>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <autoware_msgs/msg/accel_cmd.hpp>
#include <autoware_msgs/msg/steer_cmd.hpp>
#include <autoware_msgs/msg/brake_cmd.hpp>
#include <autoware_msgs/msg/lamp_cmd.hpp>
#include <autoware_msgs/msg/indicator_cmd.hpp>

#include <tablet_socket_msgs/msg/gear_cmd.hpp>
#include <tablet_socket_msgs/msg/mode_cmd.hpp>

#pragma pack(1)
#include <coms_msgs/msg/coms_control_packet.hpp>
#include <coms_msgs/msg/coms_sensor_packet.hpp>
#pragma pack()

#define MIN_COUNT (0)
#define MAX_COUNT (65535)
#define THRESHOLD ((MAX_COUNT-MIN_COUNT)/2.0)
#define mps2kmph(x) (x*3.6)
#define MAX_DIFF_TIME (20e-03)
#define COMS_WHEEL_BASE 1.53

#define STRING(var) #var

namespace Udp_ns{

class lowpassFilter
{
public:
    void setSize(int sizein);
    lowpassFilter();
    lowpassFilter(int sizein);
    void setFrequency(double cutoffFrequency, double samplingFrequency);
    double compute();
    void savePreviousInput(double xnew);

private:
    std::vector<double> x;
    std::vector<double> y;
    int size;
    double b0, b1, b2;
    double a0, a1, a2;
};


class medianFilter
{
public:
    void setSize(int sizein);
    medianFilter();
    medianFilter(int sizein);
    double compute();
    void savePreviousInput(double xnew);
  
private:
  std::vector<double> x;
  std::vector<double> y;
  int size;
};



class PlcConverter : public rclcpp::Node {
public:
    PlcConverter(std::string name);
    void run();
    void publishMsgs();
    
private:
    // time https://qiita.com/hakuturu583/items/4e559ff72c290b39e4e6
    std::shared_ptr<rclcpp::Clock> ros_clock_;
    rclcpp::Time time_;
    rclcpp::Time previous_time_;
    rclcpp::Time coms_sensor_packet_updated_time_;

    rclcpp::TimerBase::SharedPtr timer_;
    
    //Flag
    bool coms_direct_control_flag_;
    bool use_low_pass_filter_;
    bool use_median_filter_;
    int gear_num_;	//add  
   
    //Subscribers and Publishers
    //Sensor packet -> twist, pose
    rclcpp::Subscription<coms_msgs::msg::ComsSensorPacket>::SharedPtr coms_sensor_packet_sub_;

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr odom_twist_pub_;        
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr curr_pose_pub_;        
    
    //Cmd -> control packet
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_cmd_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr curr_twist_sub_;
    rclcpp::Subscription<autoware_msgs::msg::AccelCmd>::SharedPtr accel_cmd_sub_;
    rclcpp::Subscription<autoware_msgs::msg::SteerCmd>::SharedPtr steer_cmd_sub_;
    rclcpp::Subscription<autoware_msgs::msg::BrakeCmd>::SharedPtr brake_cmd_sub_;
    rclcpp::Subscription<autoware_msgs::msg::LampCmd>::SharedPtr lamp_cmd_sub_;
    rclcpp::Subscription<autoware_msgs::msg::IndicatorCmd>::SharedPtr indicator_cmd_sub_;
    rclcpp::Subscription<tablet_socket_msgs::msg::GearCmd>::SharedPtr gear_cmd_sub_;
    rclcpp::Subscription<tablet_socket_msgs::msg::ModeCmd>::SharedPtr mode_cmd_sub_;

    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr lidar_vel_sub_; // lidar vel
    rclcpp::Publisher<coms_msgs::msg::ComsControlPacket>::SharedPtr coms_control_packet_pub_;

    //Topic names
    // std::string twist_cmd_topic;
    // std::string curr_twist_topic;
    // std::string accel_cmd_topic;
    // std::string steer_cmd_topic;
    // std::string brake_cmd_topic;
    // std::string lamp_cmd_topic;
    // std::string indicator_cmd_topic;
    // std::string gear_cmd_topic;
    // std::string mode_cmd_topic;
	// std::string lidar_vel_topic;//lidar_vel

    // received messages
    coms_msgs::msg::ComsSensorPacket::SharedPtr coms_sensor_packet_msg_;
    geometry_msgs::msg::TwistStamped::SharedPtr curr_twist_msg_;

    // message to send
    geometry_msgs::msg::TwistStamped odom_twist_msg_;
    geometry_msgs::msg::PoseStamped curr_pose_msg_;
    coms_msgs::msg::ComsControlPacket coms_control_packet_msg_;

    coms_msgs::msg::ComsSensorPacket previous_coms_sensor_packet_msg_;

    //Filters
    lowpassFilter lowpass_wr;
    lowpassFilter lowpass_wl;
    medianFilter median_wr;
    medianFilter median_wl;

    //Filter parameters
    double median_filter_size_wr;
    double median_filter_size_wl;
    double lowpass_filter_cutoff_frequency_wr;
    double lowpass_filter_cutoff_frequency_wl;
    double lowpass_filter_sampling_frequency_wr;
    double lowpass_filter_sampling_frequency_wl;
    double lowpass_filter_size_wr;
    double lowpass_filter_size_wl;
    double encoder_pulse_resolution;
    double tire_radius;
    double tire_distance;

    //Filtering function
    void computeEncoderSpeed( double encoderPulseResolution, double tireRadius, double tireDistance, bool useLowpassFilter, bool useMedianFilter );

    double clipValue(double input, double min, double max);
    float clipValue(float input, float min, float max);
    int clipValue(int input, int min, int max);


    //Callback functions
    void comsSensorPacketCallback(const coms_msgs::msg::ComsSensorPacket::SharedPtr msg);
    void twistCmdCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
    void currTwistCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
    void accelCmdCallback(const autoware_msgs::msg::AccelCmd::SharedPtr msg);
    void steerCmdCallback(const autoware_msgs::msg::SteerCmd::SharedPtr msg);
    void brakeCmdCallback(const autoware_msgs::msg::BrakeCmd::SharedPtr msg);
    void lampCmdCallback(const autoware_msgs::msg::LampCmd::SharedPtr msg);
    void indicatorCmdCallback(const autoware_msgs::msg::IndicatorCmd::SharedPtr msg);
    void gearCmdCallback(const tablet_socket_msgs::msg::GearCmd::SharedPtr msg);
    void modeCmdCallback(const tablet_socket_msgs::msg::ModeCmd::SharedPtr msg);
	void LidarVelCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);//lidar_vel
};

}
#endif
