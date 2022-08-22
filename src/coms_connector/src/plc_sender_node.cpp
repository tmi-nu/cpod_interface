#include <plc_connector.h>

int main(int argc, char** argv){
    std::string destination = "192.168.13.52";
    int port = 54650;

    // private_nh.param<std::string>("destination", destination, "192.168.13.52");
    // private_nh.param<int>("port_send", port, 54650);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Udp_ns::PlcSender>("plc_sender", destination, port));
    rclcpp::shutdown();
    return 0;
}