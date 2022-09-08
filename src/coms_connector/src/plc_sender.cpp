#include <coms_connector/plc_connector.h>

namespace Udp_ns{
    
using std::placeholders::_1;
using namespace std::chrono_literals;

PlcSender::PlcSender(std::string name) 
    : Node(name)
{
    const std::string destination = declare_parameter("destination", "192.168.0.10");
    const int port =  declare_parameter("port_send", 5002);
    const int src_port = declare_parameter("port_receive", 5001);
    this->sender_ = std::make_shared<UdpSender>(destination, port, src_port);
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
    RCLCPP_INFO(this->get_logger(), "GearCmd: %d, %d", msg.get()->gear_d, msg.get()->gear_r);
    this->coms_control_packet_msg_ = msg;
    int packet_length = sizeof(*msg);
    int num = sendto(sender_->socket_var, msg.get(), packet_length, 0, (const sockaddr*)&sender_->sock_addr, sizeof(sender_->sock_addr));
    RCLCPP_INFO(this->get_logger(), "Send command: %d bytes", num);
    RCLCPP_INFO(this->get_logger(), "packet_length: %d", packet_length);
    if (num <= 0)
    {
    //   ROS_ERROR("cannot send command" );
        RCLCPP_ERROR(this->get_logger(), "cannnot send command");
    }
}

}