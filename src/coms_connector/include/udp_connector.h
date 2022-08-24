#ifndef UDP_CONNECTOR_H_
#define UDP_CONNECTOR_H_

#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <string>
#include <iostream>
#include <sstream>
#include <fstream>
#include <time.h>

#include <thread>
#include <mutex>

#define CHECKSUM_LIMIT 50 //The maximum difference in checksum not to consider as a delay in communication
#define UPDATE_LIMIT 10 //The maximum number of iteration (control/communication steps) to approve the update without change in checksum

namespace Udp_ns{
class UdpConnector{ //class for receiving and publishing the car info, subscribing and sending the control input
public:
    UdpConnector(int _port){
        port = _port;

        tv.tv_sec = 0;
        tv.tv_usec = 10000; 
        
        init();
    };   
    ~UdpConnector(){
        close(socket_var);
    };
    void run();
    int socket_var;
    struct sockaddr_in sock_addr;
    
protected:       
    int port;
    // int socket_var;
    // struct sockaddr_in sock_addr;
    struct timeval tv;
    
    void init(){
        //Setting for UDP
        memset(&sock_addr, 0, sizeof(sock_addr));
        sock_addr.sin_port = htons(port);
        sock_addr.sin_family = AF_INET;
        sock_addr.sin_addr.s_addr = htonl(INADDR_ANY);
        
        socket_var = socket(AF_INET, SOCK_DGRAM, 0);
        if(setsockopt(socket_var, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv,sizeof(struct timeval))) perror("Error in setsockopt()");
    };
};

class UdpSender : public UdpConnector{
public:
    UdpSender(std::string _destination, unsigned short _port):UdpConnector(_port){
        destination = _destination;
        
        init();
    };
    ~UdpSender(){
        close(socket_var);
    };
    // void run();
    
private:
    void init(){
        //Setting for UDP
        memset(&sock_addr, 0, sizeof(sock_addr));
        sock_addr.sin_addr.s_addr = inet_addr(destination.c_str());
        sock_addr.sin_port = htons(port);
        sock_addr.sin_family = AF_INET;
        
        socket_var = socket(AF_INET, SOCK_DGRAM, 0);
        if(setsockopt(socket_var, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv,sizeof(struct timeval))) perror("Error in setsockopt()");
    };

    std::string destination;
};

class UdpReceiver : public UdpConnector{
public:
    UdpReceiver(unsigned short _port):UdpConnector(_port){    
        init();

        bind(socket_var, (const struct sockaddr *) &sock_addr, sizeof(sock_addr));
    };
    ~UdpReceiver(){
        close(socket_var);
    };
    // void run();
};

}

#endif //UDP_CONNECTOR_H_
