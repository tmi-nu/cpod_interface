#include <plc_connector.h>

namespace Udp_ns{
    
using std::placeholders::_1;
using namespace std::chrono_literals;

PlcSender::PlcSender(std::string name, std::string _destination, unsigned short int _port) 
    : Node(name), UdpSender(_destination, _port)
{
    // this->sender_ = std::make_shared<UdpSender>(_destination, _port);
    this->coms_control_packet_sub_ = this->create_subscription<coms_msgs::msg::ComsControlPacket>
    (
        "/plc_control_packet",
        10,
        std::bind(&PlcSender::comsControlPacketCallBack, this, _1)
    );
}
    
PlcSender::~PlcSender(){
    close(this->socket_var);
}
    
void PlcSender::comsControlPacketCallBack(const coms_msgs::msg::ComsControlPacket& msg)
{
    this->coms_control_packet_msg_ = msg;
    int packet_length = sizeof(msg);
    if( sendto(socket_var, &msg, packet_length, 0, (const sockaddr*)&sock_addr, sizeof(sock_addr)) < 0 )
    {
    //   ROS_ERROR("cannot send command" );
        RCLCPP_ERROR(this->get_logger(), "cannnot send command");
    }
}

}