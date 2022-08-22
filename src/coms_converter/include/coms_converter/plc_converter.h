#ifndef PLC_CONVERTER_H_
#define PLC_CONVERTER_H_

// #include "ros/ros.h"
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/string.hpp>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <autoware_msg/msg/accel_cmd.hpp>
#include <autoware_msg/msg/steer_cmd.hpp>
#include <autoware_msg/msg/brake_cmd.hpp>
#include <autoware_msg/msg/lamp_cmd.hpp>
#include <autoware_msg/msg/indicator_cmd.hpp>

#include <tablet_socket_msgs/gear_cmd.h>
#include <tablet_socket_msgs/mode_cmd.h>

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



class PlcConverter{
public:
    PlcConverter();
    void run();
    void publishMsgs();
    
private:
    ros::Time time;
    ros::Time previous_time;
    ros::Time coms_sensor_packet_updated_time;

    //Node handles
    ros::NodeHandle nh;
    ros::NodeHandle private_nh;
    
    //Flag
    bool coms_direct_control_flag;
    bool use_low_pass_filter;
    bool use_median_filter;
    int gear_num;	//add  
   
    //Subscribers and Publishers
    //Sensor packet -> twist, pose
    ros::Subscriber coms_sensor_packet_sub;
        
    ros::Publisher odom_twist_pub;
    ros::Publisher curr_pose_pub;
    
    //Cmd -> control packet
    ros::Subscriber twist_cmd_sub;
    ros::Subscriber curr_twist_sub;
    ros::Subscriber accel_cmd_sub;
    ros::Subscriber steer_cmd_sub;
    ros::Subscriber brake_cmd_sub;
    ros::Subscriber lamp_cmd_sub;
    ros::Subscriber indicator_cmd_sub;
    ros::Subscriber gear_cmd_sub;
    ros::Subscriber mode_cmd_sub;
	ros::Subscriber lidar_vel_sub;//lidar_vel
    ros::Publisher coms_control_packet_pub;
    
    //Topic names
    std::string twist_cmd_topic;
    std::string curr_twist_topic;
    std::string accel_cmd_topic;
    std::string steer_cmd_topic;
    std::string brake_cmd_topic;
    std::string lamp_cmd_topic;
    std::string indicator_cmd_topic;
    std::string gear_cmd_topic;
    std::string mode_cmd_topic;
	std::string lidar_vel_topic;//lidar_vel

    //Topic messages
    coms_msgs::ComsSensorPacket coms_sensor_packet_msg;
    coms_msgs::ComsSensorPacket coms_sensor_packet_msg_previous;
    
    geometry_msgs::TwistStamped curr_twist_msg;
    geometry_msgs::TwistStamped odom_twist_msg;
    geometry_msgs::PoseStamped curr_pose_msg;

    coms_msgs::ComsControlPacket coms_control_packet_msg;


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
    void comsSensorPacketCallback(const coms_msgs::ComsSensorPacket& msg);
    void twistCmdCallback(const geometry_msgs::TwistStamped& msg);
    void currTwistCallback(const geometry_msgs::TwistStamped& msg);
    void accelCmdCallback(const autoware_msgs::AccelCmd& msg);
    void steerCmdCallback(const autoware_msgs::SteerCmd& msg);
    void brakeCmdCallback(const autoware_msgs::BrakeCmd& msg);
    void lampCmdCallback(const autoware_msgs::LampCmd& msg);
    void indicatorCmdCallback(const autoware_msgs::IndicatorCmd& msg);
    void gearCmdCallback(const tablet_socket_msgs::gear_cmd& msg);
    void modeCmdCallback(const tablet_socket_msgs::mode_cmd& msg);
	void LidarVelCallback(const geometry_msgs::TwistStamped& msg);//lidar_vel
};

}
#endif
