#include <coms_connector/plc_connector.h>

namespace Udp_ns{

using namespace std::chrono_literals;
    
PlcReceiver::PlcReceiver(std::string name)
    : rclcpp::Node(name)
{
    const int port = declare_parameter("port_receive", 5002);
    receiver_ = std::make_shared<UdpReceiver>(port);
    coms_sensor_packet_pub_ = this->create_publisher<coms_msgs::msg::ComsSensorPacket>
    (
        "/plc_sensor_packet",
        3
    );
    timer_ = this->create_wall_timer(
        100ms,
        std::bind(&PlcReceiver::comsSensorPacketCallBack, this)  
    );
    work_ = true;
    th_ = std::thread([this]() { this->Receiver(); });
}
        
PlcReceiver::~PlcReceiver(){
    work_ = false;
    // close(socket_var);
    receiver_->~UdpReceiver();
}
    
void PlcReceiver::comsSensorPacketCallBack(){ 
	// recv(socket_var, &coms_sensor_packet_msg_, sizeof(coms_sensor_packet_msg_), 0); 
    std::lock_guard<std::mutex> lock(mutex_);
    coms_sensor_packet_pub_->publish(coms_sensor_packet_msg_);
}
    
// void PlcReceiver::run()
// {
//     work_ = true;
//     th_ = std::thread([this]() { this->Receiver(); });
//     th_.join();
// };
  
}
