#ifndef RECEIVER_H_
#define RECEIVER_H_

#include "uav_interface.h"


#define BUF_MAX_SIZE 1024


class Receiver
{
public:
    Receiver(UAV_Interface *uav_interface_);
    ~Receiver();
    int udp_recv_init();
    int receiver_start();
    int udp_receiver();

    int recv_sockfd;                   // 接收方套接字描述符
    char buffer[BUF_MAX_SIZE];         // 数据缓冲区

private:

    struct sockaddr_in multicast_addr; // 多播组套接字信息
    struct ip_mreq multicast_group;    // 多播组信息

    UAV_Interface *_uav_interface;

};

void *udp_receiver_loop(void *args);


#endif // RECEIVER_H_