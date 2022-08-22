#include <coms_converter/plc_converter.h>

namespace Udp_ns{

using std::placeholders::_1;
using namespace std::chrono_literals;

PlcConverter::PlcConverter(std::string name) : rclcpp::Node(name)
{
    coms_direct_control_flag_ = false;
    use_low_pass_filter_ = false;
    use_median_filter_ = false;

    coms_sensor_packet_sub_ = this->create_subscription<coms_msgs::msg::ComsSensorPacket>
    (
      "/plc_sensor_packet", 1,
      std::bind(&PlcConverter::comsSensorPacketCallback, this, _1)
    );
    
    twist_cmd_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>
    (
        twist_cmd_topic, 1, std::bind(&PlcConverter::twistCmdCallback, this,_1)
    );
    curr_twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>
    (
        curr_twist_topic, 1, std::bind(&PlcConverter::currTwistCallback, this, _1)
    );
    accel_cmd_sub_ = this->create_subscription<autoware_msgs::msg::AccelCmd>
    (
        accel_cmd_topic, 1, std::bind(&PlcConverter::accelCmdCallback, this, _1)
    );
    steer_cmd_sub_ = this->create_subscription<autoware_msgs::msg::SteerCmd>
    (
        steer_cmd_topic, 1, std::bind(&PlcConverter::steerCmdCallback, this, _1)
    );
    brake_cmd_sub_ = this->create_subscription<autoware_msgs::msg::BrakeCmd>
    (
        brake_cmd_topic, 1, std::bind(&PlcConverter::brakeCmdCallback, this, _1)
    );
    lamp_cmd_sub_ = this->create_subscription<autoware_msgs::msg::LampCmd>
    (
        lamp_cmd_topic, 1, std::bind(&PlcConverter::lampCmdCallback, this, _1)
    );
    indicator_cmd_sub_ = this->create_subscription<autoware_msgs::msg::IndicatorCmd>
    (
        indicator_cmd_topic, 1, std::bind(&PlcConverter::indicatorCmdCallback, this, _1)
    );
    gear_cmd_sub_ = this->create_subscription<tablet_socket_msgs::msg::GearCmd>
    (
        gear_cmd_topic, 1, std::bind(&PlcConverter::gearCmdCallback, this, _1)
    );
    mode_cmd_sub_ = this->create_subscription<tablet_socket_msgs::msg::ModeCmd>
    (
        mode_cmd_topic, 1, std::bind(&PlcConverter::modeCmdCallback, this, _1)
    );
    lidar_vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>
    (
        lidar_vel_topic, 1, std::bind(&PlcConverter::LidarVelCallback, this, _1)
    );

    odom_twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/odom_twist", 1);
    curr_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/curr_pose", 1);
    coms_control_packet_pub_ = this->create_publisher<coms_msgs::msg::ComsControlPacket>("/plc_control_packet", 1);

    //Setup filters
    if(use_median_filter_){
        median_wr.setSize(median_filter_size_wr);
        median_wl.setSize(median_filter_size_wl);
    }
    if(use_low_pass_filter_){
        lowpass_wr.setFrequency(lowpass_filter_cutoff_frequency_wr, lowpass_filter_sampling_frequency_wr);
        lowpass_wl.setFrequency(lowpass_filter_cutoff_frequency_wl, lowpass_filter_sampling_frequency_wl);
        lowpass_wr.setSize(lowpass_filter_size_wr);
        lowpass_wl.setSize(lowpass_filter_size_wl);
    }

    ros_clock_ = std::make_shared<rclcpp::Clock>(rcl_clock_type_t::RCL_ROS_TIME);
    rclcpp::Time now = ros_clock_->now();
    previous_time_ = now;
    coms_sensor_packet_updated_time_ = now;   

    timer_ = this->create_wall_timer(
        100ms,
        std::bind(&PlcConverter::publishMsgs, this)
    );
}

    
void PlcConverter::publishMsgs(){
    if (coms_control_packet_msg_.wd_count > 10000) coms_control_packet_msg_.wd_count = 0;
    else coms_control_packet_msg_.wd_count++;

    odom_twist_pub_->publish(odom_twist_msg_);
    curr_pose_pub_->publish(curr_pose_msg_);
    coms_control_packet_pub_->publish(coms_control_packet_msg_);
}
    
void PlcConverter::comsSensorPacketCallback(const coms_msgs::msg::ComsSensorPacket& msg){
    coms_sensor_packet_msg_ = msg;
    coms_sensor_packet_updated_time_ = ros_clock_->now();

    computeEncoderSpeed(encoder_pulse_resolution, tire_radius, tire_distance, use_low_pass_filter_, use_median_filter_);
}

void PlcConverter::twistCmdCallback(const geometry_msgs::msg::TwistStamped& msg){
    if(!coms_direct_control_flag_){
        coms_control_packet_msg_.acc_vel_ref = msg.twist.linear.x;
    
        // Yaw rate -> steer angle (tire angle)
        // Put the value only when the velocity is not 0.
        // Do not set any value on tir_ang_ref when the velocity is 0 and just keep the previous value.
        if(msg.twist.linear.x > 0.1){
            float tmp;
            tmp = COMS_WHEEL_BASE * msg.twist.angular.z / msg.twist.linear.x;
            tmp = asin(tmp);
            tmp = clipValue(tmp, static_cast<float>(-60.0f/180.0f*M_PI), static_cast<float>(60.0f/180.0f*M_PI));

            coms_control_packet_msg_.tir_ang_ref = tmp;
        }
    }
}

void PlcConverter::currTwistCallback(const geometry_msgs::msg::TwistStamped& msg){
    curr_twist_msg_ = msg;
}
  
void PlcConverter::accelCmdCallback(const autoware_msgs::msg::AccelCmd& msg){
    if(coms_direct_control_flag_){
	    int tmp = clipValue((int)msg.accel,0,20);
	    coms_control_packet_msg_.acc_vel_ref = (float)tmp/3.6f; 
    }
}

void PlcConverter::steerCmdCallback(const autoware_msgs::msg::SteerCmd& msg){
    if(coms_direct_control_flag_){
        float tmp = clipValue((float)msg.steer/17.5f, -35.0f, 35.0f); //steering wheel angle -> tire angle
	    coms_control_packet_msg_.tir_ang_ref = (float)tmp*M_PI/180.0f; //degree -> radian
    }
}

void PlcConverter::brakeCmdCallback(const autoware_msgs::msg::BrakeCmd& msg){
    if(coms_direct_control_flag_){
	    int tmp = clipValue((int)msg.brake,0,100);
	    coms_control_packet_msg_.brk_pos_ref = tmp;
    }
}

void PlcConverter::lampCmdCallback(const autoware_msgs::msg::LampCmd& msg){
    coms_control_packet_msg_.acc_cmode = msg.l?2:0; //int32
	coms_control_packet_msg_.brk_cmode = msg.l?3:0;
	    
    coms_control_packet_msg_.tir_cmode = msg.r?3:0; //int32
}

void PlcConverter::indicatorCmdCallback(const autoware_msgs::msg::IndicatorCmd& msg){
    coms_control_packet_msg_.winker_l = msg.l; //int32
    coms_control_packet_msg_.winker_r = msg.r; //int32
}
    
void PlcConverter::gearCmdCallback(const tablet_socket_msgs::msg::GearCmd& msg){
    int tmp = msg.gear; //int32
    switch(tmp){
        case 1: //D
            coms_control_packet_msg_.gear_d = 1;
            coms_control_packet_msg_.gear_r = 0;
            break;
        case 2: //R
            coms_control_packet_msg_.gear_d = 0;
            coms_control_packet_msg_.gear_r = 1;
            break;
        case 3: //B (but set as 0,0)
            coms_control_packet_msg_.gear_d = 0;
            coms_control_packet_msg_.gear_r = 0;
            break;
        case 4: //N
            coms_control_packet_msg_.gear_d = 1;
            coms_control_packet_msg_.gear_r = 1;
            break;
    }
}    

void PlcConverter::modeCmdCallback(const tablet_socket_msgs::msg::ModeCmd& msg){
    coms_control_packet_msg_.auto_control = msg.mode; //int32
}

void PlcConverter::LidarVelCallback(const geometry_msgs::msg::TwistStamped& msg){
	    coms_control_packet_msg_.pc_speed_mps = msg.twist.linear.x;// lidar_vel
}

double PlcConverter::clipValue(double input, double min, double max){
    double output = input;    
    output = (output>max ? max : output);
    output = (output<min ? min : output);

    return output;
}

float PlcConverter::clipValue(float input, float min, float max){
    float output = input;    
    output = (output>max ? max : output);
    output = (output<min ? min : output);

    return output;
}

int PlcConverter::clipValue(int input, int min, int max){
    int output = input;    
    output = (output>max ? max : output);
    output = (output<min ? min : output);

    return output;
}

void PlcConverter::computeEncoderSpeed( double encoderPulseResolution, double tireRadius, double tireDistance, bool useLowpassFilter, bool useMedianFilter){
    //Pool sensor
    coms_msgs::msg::ComsSensorPacket coms_sensor_packet_now;
	    
    coms_sensor_packet_now = coms_sensor_packet_msg_;
 	    
    rclcpp::Time now = coms_sensor_packet_updated_time_;
  
    //Get time difference
    uint64_t diff_time_nsec = now.nanoseconds() - previous_time_.nanoseconds();
    double diff_time_sec = diff_time_nsec*1e-09;

    double dt = diff_time_sec;  
    if( dt > MAX_DIFF_TIME ){ 
    previous_time_ = now;
  
    //Get pulse and calculate difference
    double pulse_rr;
    double pulse_rl;
    double d_pulse_rr;
    double d_pulse_rl;

    pulse_rr = coms_sensor_packet_now.r_tire;
    pulse_rl = coms_sensor_packet_now.l_tire;
    d_pulse_rr = coms_sensor_packet_now.r_tire - previous_coms_sensor_packet_msg_.r_tire;
    d_pulse_rl = coms_sensor_packet_now.l_tire - previous_coms_sensor_packet_msg_.l_tire;

    if (d_pulse_rr < -THRESHOLD ){ d_pulse_rr += MAX_COUNT-MIN_COUNT; }
    if (d_pulse_rr >  THRESHOLD ){ d_pulse_rr -= MAX_COUNT-MIN_COUNT; }
    if (d_pulse_rl < -THRESHOLD ){ d_pulse_rl += MAX_COUNT-MIN_COUNT; }
    if (d_pulse_rl >  THRESHOLD ){ d_pulse_rl -= MAX_COUNT-MIN_COUNT; }

    //Compute wheel rotation speed
    double wr = d_pulse_rr * 2*M_PI /(encoderPulseResolution*dt) ;
    double wl = d_pulse_rl * 2*M_PI /(encoderPulseResolution*dt) ;

    double wr_raw = wr;
    double wl_raw = wl;

    //Apply filters
    if( use_median_filter_ ){
        median_wr.savePreviousInput( wr );
        wr = median_wr.compute();     
        median_wl.savePreviousInput( wl );
        wl = median_wl.compute();	  
    }

    if( use_low_pass_filter_ ){
        lowpass_wr.savePreviousInput( wr );
        wr = lowpass_wr.compute();     
        lowpass_wl.savePreviousInput( wl );
        wl = lowpass_wl.compute();	  
    }

    //Compute wheel speed
    double vrr = tireRadius*wr;//m/sec
    double vrl = tireRadius*wl;//m/sec

    double vrr_raw = tireRadius*wr_raw;//m/sec
    double vrl_raw = tireRadius*wl_raw;//m/sec
  
    double v = clipValue((vrr + vrl)*0.5, 0.0, 60.0/3.6);
    double w = (-vrl + vrr)*0.5/(0.5*tireDistance);

    double v_raw = (vrr_raw + vrl_raw)*0.5;
    double w_raw = (-vrl_raw + vrr_raw)*0.5/(0.5*tireDistance);

    //Pack the calculated values into the messages
    odom_twist_msg_.header.stamp = ros_clock_->now();
    //odom_twist_msg.header.frame_id = "/base_link";
    odom_twist_msg_.twist.linear.x = v;
    odom_twist_msg_.twist.linear.y = 0.0;
    odom_twist_msg_.twist.linear.z = 0.0;
    odom_twist_msg_.twist.angular.x = 0.0;
    odom_twist_msg_.twist.angular.y = 0.0;
    odom_twist_msg_.twist.angular.z = w;

    //Pack the velocity into the control packet also
    //coms_control_packet_msg.pc_speed_mps = v;
    coms_control_packet_msg_.pc_yawrate = w;
    coms_control_packet_msg_.pc_speedr_mps = vrr;
    coms_control_packet_msg_.pc_speedl_mps = vrl;

	previous_coms_sensor_packet_msg_ = coms_sensor_packet_now;
	    
    }
}

void lowpassFilter::setSize(int sizein){
    if( sizein < 1 ){
      sizein = 1;
    }
    size = sizein;
    
    x.resize(size);
    y.resize(size);
    for( int n=0; n < size; n++ ){
      x[n] = 0.0;
    }
    for( int n=0; n < size; n++ ){
      y[n] = 0.0;
    }
}
  
lowpassFilter::lowpassFilter(){
    setSize(10);
    b0 = 0.0; b1 = 0.0; b2 = 0.0; 
    a0 = 0.0; a1 = 0.0; a2 = 0.0;
}
  
lowpassFilter::lowpassFilter(int sizein){
    setSize(sizein);
    b0 = 0.0; b1 = 0.0; b2 = 0.0; 
    a0 = 0.0; a1 = 0.0; a2 = 0.0;
}

void lowpassFilter::setFrequency( double cutoffFrequency, double samplingFrequency ){
    double fd = cutoffFrequency;
    double fs = samplingFrequency;
    double q = 1/sqrt(2.0);
    
    //filter coefficient in time domain    
    double omega = 2.0*M_PI*fd/fs;
    double alpha = sin(omega)/(2.0*q);
    a0 =  1.0 + alpha;
    a1 = -2.0 * cos(omega);
    a2 =  1.0 - alpha;
    b0 = (1.0 - cos(omega)) / 2.0;
    b1 =  1.0 - cos(omega);
    b2 = (1.0 - cos(omega)) / 2.0;
}
  
double lowpassFilter::compute(){
    
    double in1  = 0.0;
    double in2  = 0.0;
    double out1 = 0.0;
    double out2 = 0.0;
    
    for(int i = 0; i < size; i++){
      y[i] = b0/a0 * x[i] + b1/a0 * in1  + b2/a0 * in2
	- a1/a0 * out1 - a2/a0 * out2;
      
      in2  = in1;
      in1  = x[i]; 
      out2 = out1;     
      out1 = y[i];      
    }
    
    return y[size-1];
}
  
void lowpassFilter::savePreviousInput(double xnew){
    for( int i = 0; i < size-1; i++ ){
      x[i] = x[i+1];
    }
    x[size-1] = xnew;
}


void medianFilter::setSize(int sizein){
    if( sizein < 1 ){
      sizein = 1;
    }
    size = sizein;
    
    x.resize(size);
    y.resize(size);
    for( int n=0; n < size; n++ ){
      x[n] = 0.0;
    }
    for( int n=0; n < size; n++ ){
      y[n] = 0.0;
    }
}
  
medianFilter::medianFilter(){
    setSize(5);
}
  
medianFilter::medianFilter(int sizein){
    setSize(sizein);
}

double medianFilter::compute(){    
    std::vector<double> xSorted = x; 
    std::sort(xSorted.begin(), xSorted.end());
    
    int center = floor(size/2.0);
    if( center > size ){ center = size; }
    if( center < 0 ){ center = 0; }
    
    return xSorted[center];
}
  
void medianFilter::savePreviousInput(double xnew){
    for( int i = 0; i < size-1; i++ ){
      x[i] = x[i+1];
    }
    x[size-1] = xnew;
}




}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Udp_ns::PlcConverter>("plc_converter"));
    rclcpp::shutdown();

    // ros::init(argc, argv, "plc_converter");
    // Udp_ns::PlcConverter plc_converter;
    // plc_converter.run();
}
