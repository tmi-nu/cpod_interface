#include <coms_connector/plc_connector.h>

namespace Udp_ns{
    
using std::placeholders::_1;
using namespace std::chrono_literals;

PlcSender::PlcSender(std::string name) 
    : Node(name)
{
    const std::string destination = declare_parameter("destination", "192.168.0.20");
    const int port =  declare_parameter("port", 5002);
    this->sender_ = std::make_shared<UdpSender>(destination, port);
    this->coms_control_packet_sub_ = this->create_subscription<coms_msgs::msg::ComsControlPacket>
    (
        "/plc_control_packet",
        10,
        std::bind(&PlcSender::comsControlPacketCallBack, this, _1)
    );
}
    
PlcSender::~PlcSender(){
    // close(this->socket_var);
    sender_->~UdpSender();
}
    
void PlcSender::comsControlPacketCallBack(const coms_msgs::msg::ComsControlPacket::SharedPtr msg)
{
    this->coms_control_packet_msg_ = msg;
    int packet_length = sizeof(msg);
    if( sendto(sender_->socket_var, &msg, packet_length, 0, (const sockaddr*)&sender_->sock_addr, sizeof(sender_->sock_addr)) < 0 )
    {
    //   ROS_ERROR("cannot send command" );
        RCLCPP_ERROR(this->get_logger(), "cannnot send command");
    }
}

}