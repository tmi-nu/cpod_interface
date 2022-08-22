#include <plc_converter.h>

namespace Udp_ns{
PlcConverter::PlcConverter():private_nh("~"){
    //Setup flags
    coms_direct_control_flag = false;
    use_low_pass_filter = false;
    use_median_filter =false;

    //Read parameters
    private_nh.param<bool>("direct_control_flag", coms_direct_control_flag, false);
    private_nh.param<bool>( "use_low_pass_filter", use_low_pass_filter, false );
    private_nh.param<bool>( "use_median_filter", use_median_filter, false );
    private_nh.param<double>( "lowpass_filter_cutoff_frequency_wr", lowpass_filter_cutoff_frequency_wr, 50.0 );
    private_nh.param<double>( "lowpass_filter_cutoff_frequency_wl", lowpass_filter_cutoff_frequency_wl, 50.0 );
    private_nh.param<double>( "lowpass_filter_sampling_frequency_wr", lowpass_filter_sampling_frequency_wr, 100.0 );
    private_nh.param<double>( "lowpass_filter_sampling_frequency_wl", lowpass_filter_sampling_frequency_wl, 100.0 );
    private_nh.param<double>( "lowpass_filter_size_wr", lowpass_filter_size_wr, 20 );
    private_nh.param<double>( "lowpass_filter_size_wl", lowpass_filter_size_wl, 20 );
    private_nh.param<double>( "median_filter_size_wr", median_filter_size_wr, 5 );
    private_nh.param<double>( "median_filter_size_wl", median_filter_size_wl, 5 );
    private_nh.param<double>( "tire_radius", tire_radius, 1.0 );
    private_nh.param<double>( "tire_distance", tire_distance, 1.0 );
    private_nh.param<double>( "encoder_pulse_resolution", encoder_pulse_resolution, 1.0 );

    private_nh.param<std::string>("twist_cmd_topic", twist_cmd_topic, "/twist_cmd");
    private_nh.param<std::string>("curr_twist_topic", curr_twist_topic, "/curr_twist");
    private_nh.param<std::string>("accel_cmd_topic", accel_cmd_topic, "/accel_cmd");
    private_nh.param<std::string>("steer_cmd_topic", steer_cmd_topic, "/steer_cmd");
    private_nh.param<std::string>("brake_cmd_topic", brake_cmd_topic, "/brake_cmd");
    private_nh.param<std::string>("lamp_cmd_topic", lamp_cmd_topic, "/lamp_cmd");
    private_nh.param<std::string>("indicator_cmd_topic", indicator_cmd_topic, "/indicator_cmd");
    private_nh.param<std::string>("gear_cmd_topic", gear_cmd_topic, "/gear_cmd");
    private_nh.param<std::string>("mode_cmd_topic", mode_cmd_topic, "/mode_cmd");
	private_nh.param<std::string>("lidar_vel_topic", lidar_vel_topic, "/estimate_twist");//lidar_vel
    

    //Setup subscribers
	coms_sensor_packet_sub = nh.subscribe("/plc_sensor_packet", 1,     
&PlcConverter::comsSensorPacketCallback, this);

    twist_cmd_sub = nh.subscribe(twist_cmd_topic,1,&PlcConverter::twistCmdCallback, this);
    curr_twist_sub = nh.subscribe(curr_twist_topic,1,&PlcConverter::currTwistCallback, this);
    accel_cmd_sub = nh.subscribe(accel_cmd_topic,1,&PlcConverter::accelCmdCallback, this);
    steer_cmd_sub = nh.subscribe(steer_cmd_topic,1,&PlcConverter::steerCmdCallback, this);
    brake_cmd_sub = nh.subscribe(brake_cmd_topic,1,&PlcConverter::brakeCmdCallback, this);
    lamp_cmd_sub = nh.subscribe(lamp_cmd_topic,1,&PlcConverter::lampCmdCallback, this);
    indicator_cmd_sub = nh.subscribe(indicator_cmd_topic,1,&PlcConverter::indicatorCmdCallback, this);
    gear_cmd_sub = nh.subscribe(gear_cmd_topic,1,&PlcConverter::gearCmdCallback, this);
    mode_cmd_sub = nh.subscribe(mode_cmd_topic,1,&PlcConverter::modeCmdCallback, this);
	lidar_vel_sub = nh.subscribe(lidar_vel_topic,1,&PlcConverter::LidarVelCallback,this);//lidar_vel
    
    //Setup publishers
    odom_twist_pub = nh.advertise<geometry_msgs::TwistStamped>("/odom_twist", 1);
    curr_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/curr_pose", 1);
    coms_control_packet_pub = nh.advertise<coms_msgs::ComsControlPacket>("/plc_control_packet", 1);
    
   
    //Setup filters
    if(use_median_filter){
        median_wr.setSize(median_filter_size_wr);
        median_wl.setSize(median_filter_size_wl);
    }
    if(use_low_pass_filter){
        lowpass_wr.setFrequency(lowpass_filter_cutoff_frequency_wr, lowpass_filter_sampling_frequency_wr);
        lowpass_wl.setFrequency(lowpass_filter_cutoff_frequency_wl, lowpass_filter_sampling_frequency_wl);
        lowpass_wr.setSize(lowpass_filter_size_wr);
        lowpass_wl.setSize(lowpass_filter_size_wl);
    }

    //Setup variables
    ros::Time now = ros::Time::now();
    previous_time = now;
    coms_sensor_packet_updated_time = now;
};

void PlcConverter::run(){
    ros::Rate loop_rate(100);
    while(ros::ok()){
        private_nh.param<bool>("direct_control_flag", coms_direct_control_flag, false);
        ros::spinOnce();

        if(coms_control_packet_msg.wd_count > 10000) coms_control_packet_msg.wd_count = 0;
        else coms_control_packet_msg.wd_count++;

        publishMsgs();
        //Need to add time-out function
        loop_rate.sleep();
    }
};
    
void PlcConverter::publishMsgs(){
     odom_twist_pub.publish(odom_twist_msg);
     curr_pose_pub.publish(curr_pose_msg);
     coms_control_packet_pub.publish(coms_control_packet_msg);
};
    
void PlcConverter::comsSensorPacketCallback(const coms_msgs::ComsSensorPacket& msg){
    coms_sensor_packet_msg = msg;
    coms_sensor_packet_updated_time = ros::Time::now();

    computeEncoderSpeed(encoder_pulse_resolution, tire_radius, tire_distance, use_low_pass_filter, use_median_filter);
};

void PlcConverter::twistCmdCallback(const geometry_msgs::TwistStamped& msg){
    if(!coms_direct_control_flag){
        coms_control_packet_msg.acc_vel_ref = msg.twist.linear.x;
    
        // Yaw rate -> steer angle (tire angle)
        // Put the value only when the velocity is not 0.
        // Do not set any value on tir_ang_ref when the velocity is 0 and just keep the previous value.
        if(msg.twist.linear.x > 0.1){
            float tmp;
            tmp = COMS_WHEEL_BASE * msg.twist.angular.z / msg.twist.linear.x;
            tmp = asin(tmp);
            tmp = clipValue(tmp, (float)-60.0f/180.0f*M_PI, (float)60.0f/180.0f*M_PI);

            coms_control_packet_msg.tir_ang_ref = tmp;
        }
    }
};

void PlcConverter::currTwistCallback(const geometry_msgs::TwistStamped& msg){
    curr_twist_msg = msg;
};
  
void PlcConverter::accelCmdCallback(const autoware_msgs::AccelCmd& msg){
    if(coms_direct_control_flag){
	    int tmp = clipValue((int)msg.accel,0,20);
	    coms_control_packet_msg.acc_vel_ref = (float)tmp/3.6f; 
    }
};

void PlcConverter::steerCmdCallback(const autoware_msgs::SteerCmd& msg){
    if(coms_direct_control_flag){
        float tmp = clipValue((float)msg.steer/17.5f, -35.0f, 35.0f); //steering wheel angle -> tire angle
	    coms_control_packet_msg.tir_ang_ref = (float)tmp*M_PI/180.0f; //degree -> radian
    }
};

void PlcConverter::brakeCmdCallback(const autoware_msgs::BrakeCmd& msg){
    if(coms_direct_control_flag){
	    int tmp = clipValue((int)msg.brake,0,100);
	    coms_control_packet_msg.brk_pos_ref = tmp;
    }
};

void PlcConverter::lampCmdCallback(const autoware_msgs::LampCmd& msg){
    coms_control_packet_msg.acc_cmode = msg.l?2:0; //int32
	coms_control_packet_msg.brk_cmode = msg.l?3:0;
	    
    coms_control_packet_msg.tir_cmode = msg.r?3:0; //int32
};

void PlcConverter::indicatorCmdCallback(const autoware_msgs::IndicatorCmd& msg){
    coms_control_packet_msg.winkerL = msg.l; //int32
    coms_control_packet_msg.winkerR = msg.r; //int32
};
    
void PlcConverter::gearCmdCallback(const tablet_socket_msgs::gear_cmd& msg){
    int tmp = msg.gear; //int32
    switch(tmp){
        case 1: //D
            coms_control_packet_msg.gearD = 1;
            coms_control_packet_msg.gearR = 0;
            break;
        case 2: //R
            coms_control_packet_msg.gearD = 0;
            coms_control_packet_msg.gearR = 1;
            break;
        case 3: //B (but set as 0,0)
            coms_control_packet_msg.gearD = 0;
            coms_control_packet_msg.gearR = 0;
            break;
        case 4: //N
            coms_control_packet_msg.gearD = 1;
            coms_control_packet_msg.gearR = 1;
            break;
    }
};    

void PlcConverter::modeCmdCallback(const tablet_socket_msgs::mode_cmd& msg){
    coms_control_packet_msg.auto_control = msg.mode; //int32
};

void PlcConverter::LidarVelCallback(const geometry_msgs::TwistStamped& msg){
	    coms_control_packet_msg.pc_speed_mps = msg.twist.linear.x;// lidar_vel
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
    coms_msgs::ComsSensorPacket coms_sensor_packet_now;
	    
    coms_sensor_packet_now = coms_sensor_packet_msg;
 	    
    ros::Time now = coms_sensor_packet_updated_time;
  
    //Get time difference
    uint64_t diff_time_nsec = now.toNSec() - previous_time.toNSec();
    double diff_time_sec = diff_time_nsec*1e-09;

    double dt = diff_time_sec;  
    if( dt > MAX_DIFF_TIME ){ 
    previous_time = now;
  
    //Get pulse and calculate difference
    double pulse_rr;
    double pulse_rl;
    double d_pulse_rr;
    double d_pulse_rl;

    pulse_rr = coms_sensor_packet_now.R_Tire;
    pulse_rl = coms_sensor_packet_now.L_Tire;
    d_pulse_rr = coms_sensor_packet_now.R_Tire - coms_sensor_packet_msg_previous.R_Tire;
    d_pulse_rl = coms_sensor_packet_now.L_Tire - coms_sensor_packet_msg_previous.L_Tire;

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
    if( use_median_filter ){
        median_wr.savePreviousInput( wr );
        wr = median_wr.compute();     
        median_wl.savePreviousInput( wl );
        wl = median_wl.compute();	  
    }

    if( use_low_pass_filter ){
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
    odom_twist_msg.header.stamp = ros::Time::now();
    //odom_twist_msg.header.frame_id = "/base_link";
    odom_twist_msg.twist.linear.x = v;
    odom_twist_msg.twist.linear.y = 0.0;
    odom_twist_msg.twist.linear.z = 0.0;
    odom_twist_msg.twist.angular.x = 0.0;
    odom_twist_msg.twist.angular.y = 0.0;
    odom_twist_msg.twist.angular.z = w;

    //Pack the velocity into the control packet also
    //coms_control_packet_msg.pc_speed_mps = v;
    coms_control_packet_msg.pc_yawrate = w;
    coms_control_packet_msg.pc_speedr_mps = vrr;
    coms_control_packet_msg.pc_speedl_mps = vrl;

	coms_sensor_packet_msg_previous = coms_sensor_packet_now;
	    
    }
};

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
};
  
lowpassFilter::lowpassFilter(){
    setSize(10);
    b0 = 0.0; b1 = 0.0; b2 = 0.0; 
    a0 = 0.0; a1 = 0.0; a2 = 0.0;
};
  
lowpassFilter::lowpassFilter(int sizein){
    setSize(sizein);
    b0 = 0.0; b1 = 0.0; b2 = 0.0; 
    a0 = 0.0; a1 = 0.0; a2 = 0.0;
};

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
};
  
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
};
  
void lowpassFilter::savePreviousInput(double xnew){
    for( int i = 0; i < size-1; i++ ){
      x[i] = x[i+1];
    }
    x[size-1] = xnew;
};


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
};
  
medianFilter::medianFilter(){
    setSize(5);
};
  
medianFilter::medianFilter(int sizein){
    setSize(sizein);
};

double medianFilter::compute(){    
    std::vector<double> xSorted = x; 
    std::sort(xSorted.begin(), xSorted.end());
    
    int center = floor(size/2.0);
    if( center > size ){ center = size; }
    if( center < 0 ){ center = 0; }
    
    return xSorted[center];
};
  
void medianFilter::savePreviousInput(double xnew){
    for( int i = 0; i < size-1; i++ ){
      x[i] = x[i+1];
    }
    x[size-1] = xnew;
};




}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "plc_converter");
    
    Udp_ns::PlcConverter plc_converter;
    plc_converter.run();
}
