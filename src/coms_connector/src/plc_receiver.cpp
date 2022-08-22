#include <plc_connector.h>

namespace Udp_ns{

using namespace std::chrono_literals;
    
PlcReceiver::PlcReceiver(std::string name, unsigned short _port)
    : rclcpp::Node(name), UdpReceiver(_port)
{
    work_ = true;
    th_ = std::thread([this]() { this->Receiver(); });
    th_.join();
    coms_sensor_packet_pub_ = this->create_publisher<coms_msgs::msg::ComsSensorPacket>
    (
        "/plc_sensor_packet",
        3
    );
    timer_ = this->create_wall_timer(
        100ms,
        std::bind(&PlcReceiver::comsSensorPacketCallBack, this)  
    );
}
        
PlcReceiver::~PlcReceiver(){
    work_ = false;
    close(socket_var);
}
    
void PlcReceiver::comsSensorPacketCallBack(){ 
	// recv(socket_var, &coms_sensor_packet_msg_, sizeof(coms_sensor_packet_msg_), 0); 
    {
        std::lock_guard<std::mutex> lock(mutex_);
        coms_sensor_packet_pub_->publish(coms_sensor_packet_msg_);
    }
}
    
// void PlcReceiver::run()
// {
//     work_ = true;
//     th_ = std::thread([this]() { this->Receiver(); });
//     th_.join();
// };
  
}
