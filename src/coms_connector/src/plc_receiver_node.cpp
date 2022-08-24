#include <plc_connector.h>



int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Udp_ns::PlcReceiver>("plc_reciver"));
    rclcpp::shutdown();

    return 0;
}