#include <plc_connector.h>

int main(int argc, char** argv){
    // ros::init(argc, argv, "plc_receiver");

    int port = 54650;

    // ros::NodeHandle private_nh("~");
    
    // private_nh.param<int>("port_receive", port, 54650);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Udp_ns::PlcReceiver>("plc_reciver", port));
    rclcpp::shutdown();

    // Udp_ns::PlcReceiver plc_receiver("plc_reciver", port);
    // plc_receiver.run();    
    
    return 0;
}
