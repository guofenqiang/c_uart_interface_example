#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include "sender.h"


Sender::Sender(UAV_Interface *uav_interface_)
{
    _uav_interface = uav_interface_;
}

Sender::~Sender()
{
    close(sockfd);
}

int Sender::udp_sender() {
    if (_uav_interface->_ptconv->send_len >= 12) {
        // for (int i = 0; i < port->send_len; i++) {
        //     printf("%02x ", port->send_buff[i]);
        // }
        // printf("\n");

        // 发送组播数据
        if (sendto(sockfd, _uav_interface->_ptconv->send_buff, _uav_interface->_ptconv->send_len, 0, (struct sockaddr *) &multicast_addr, sizeof(multicast_addr)) < 0)
        {
            perror("sendto");
            exit(EXIT_FAILURE);
        };
    }


    return 0;
}

void *udp_sender_loop(void *args)
{
	Sender *sender = (Sender *)args;
    while (1) {
        int result = sender->udp_sender();
        usleep(200000);
    }

	return NULL;
}

int Sender::send_start()
{
	pthread_t read_tid;
	int result;

	result = pthread_create( &read_tid, NULL, udp_sender_loop, this);
	if ( result ) throw result;
}

int Sender::udp_send_init()
{
#define MULTICAST_IP "224.0.0.2" // 组播IP地址
#define PORT 7043 // 端口号
#define IF_NAME "eth0" // 网口名称

    // 创建UDP套接字
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        perror("socket");
        exit(EXIT_FAILURE);
    }

    // 设置网口
    if (setsockopt(sockfd, SOL_SOCKET, SO_BINDTODEVICE, IF_NAME, strlen(IF_NAME)) < 0) {
        perror("setsockopt");
        exit(EXIT_FAILURE);
    }

    // 设置组播IP和端口
    memset(&multicast_addr, 0, sizeof(multicast_addr));
    multicast_addr.sin_family = AF_INET;
    multicast_addr.sin_port = htons(PORT);
    if (inet_pton(AF_INET, MULTICAST_IP, &(multicast_addr.sin_addr)) <= 0) {
        perror("inet_pton");
        exit(EXIT_FAILURE);
    }

    send_start();

    return 0;
}
