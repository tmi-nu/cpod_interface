#ifndef PLC_CONNECTOR_H_
#define PLC_CONNECTOR_H_

#include "udp_connector.h"
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <chrono>

#pragma pack(1)
#include <coms_msgs/msg/coms_control_packet.hpp>
#include <coms_msgs/msg/coms_sensor_packet.hpp>
#pragma pack()

namespace Udp_ns{

class PlcSender : public rclcpp::Node, public UdpSender {
    public:
        PlcSender(std::string name, std::string _destination, unsigned short _port);
        ~PlcSender();
        // void run();

    private:
        // std::shared_ptr<UdpSender> sender_;
        coms_msgs::msg::ComsControlPacket coms_control_packet_msg_;
        rclcpp::Subscription<coms_msgs::msg::ComsControlPacket>::SharedPtr coms_control_packet_sub_;
        void comsControlPacketCallBack(const coms_msgs::msg::ComsControlPacket& msg);
        // void comsControlPacketSend(const coms_msgs::msg::ComsControlPacket& msg);
};

class PlcReceiver : public rclcpp::Node, public UdpReceiver {
    public:
        PlcReceiver(std::string name, unsigned short int _port);
        ~PlcReceiver();
        // void run();

    private:
        // std::shared_ptr<UdpReceiver> receiver_;
        coms_msgs::msg::ComsSensorPacket coms_sensor_packet_msg_;
        rclcpp::Publisher<coms_msgs::msg::ComsSensorPacket> ::SharedPtr coms_sensor_packet_pub_;
        rclcpp::TimerBase::SharedPtr timer_;
        void comsSensorPacketCallBack();

        std::thread th_;
        std::mutex mutex_;
        bool work_;
        int interval_;

        void Receiver()
        {
            interval_ = 750;
            timeval timeout_time;

            timeout_time.tv_sec = 1;
            timeout_time.tv_usec = 0;
            fd_set fds, readfds;

            FD_ZERO(&readfds);
            FD_SET(socket_var, &readfds);

            while (work_)
            {
                memcpy(&fds, &readfds, sizeof(fd_set));

                if (select(socket_var + 1, &fds, NULL, NULL, &timeout_time) < 1
                    || !FD_ISSET(socket_var, &fds)) {
                    usleep(interval_);
                    continue;
                }

                auto data_size = recv(socket_var, (void *) (&coms_sensor_packet_msg_), sizeof(coms_sensor_packet_msg_), 0);
                //std::cout << data_size << std::endl;
            }
        }
};


}
#endif //PLC_CONNECTOR_H_
