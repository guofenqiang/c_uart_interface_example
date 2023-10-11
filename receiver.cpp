#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include "receiver.h"

 
 Receiver::Receiver(UAV_Interface *uav_interface_)
 {
    _uav_interface = uav_interface_;
 }
 Receiver::~Receiver()
 {

 }

int Receiver::udp_receiver() {
    /* 循环接收多播消息 */
    size_t total_bytes;
    char buffer_last[BUF_MAX_SIZE]; 

    memset(buffer, 0, BUF_MAX_SIZE);
    total_bytes = recvfrom(recv_sockfd, buffer, sizeof(buffer), 0, NULL, 0);
    if (!_uav_interface->compareArrays(buffer, buffer_last, total_bytes)) {
        memcpy(buffer_last, buffer, total_bytes);
        for(int i = 0; i < total_bytes; i++) {
        printf("%02x", buffer[i]);
        }
        printf("\n");
        _uav_interface->_ptconv->bz_telecontrol_decode(buffer, total_bytes);       
    } else {
        usleep(10000);
    }

    return 0;
}

void *udp_receiver_loop(void *args)
{
    Receiver *receiver = (Receiver *)args;
    while (1) {
        int result = receiver->udp_receiver();

    }

	return NULL;
}

int Receiver::receiver_start()
{
	pthread_t read_tid;
	int result;

	result = pthread_create( &read_tid, NULL, udp_receiver_loop, this);
	if ( result ) throw result;
}

int Receiver::udp_recv_init()
{
    pthread_t read_tid;
    int result;

    int argc = 3;
    char *argv[] = {
        {""},
        {"224.0.0.3"},
        {"7044"}
    };

    /* 参数检查 */
    if (argc != 3) {
        printf("Command format : %s <multicast IP> <port>!\n", argv[0]);
        exit(1);
    }

    /* 获取接收方套接字文件描述符 */
    if ((recv_sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("socket error");
        exit(1);
    }

    /* 初始化多播组套接字信息 */
    memset(&multicast_addr, 0, sizeof(struct sockaddr_in));
    multicast_addr.sin_family = AF_INET;                 // 使用IPv4
    multicast_addr.sin_addr.s_addr = inet_addr(argv[1]); // 指定多播组IP
    multicast_addr.sin_port = htons(atoi(argv[2]));      // 对端口进行字节序转化

    /* 绑定多播组套接字信息 */
    if (bind(recv_sockfd, (struct sockaddr *) (&multicast_addr), sizeof(struct sockaddr_in)) < 0) {
        perror("bind error");
        exit(1);
    }

    /* 将本机加入多播组 */
    multicast_group.imr_multiaddr.s_addr = inet_addr(argv[1]); // 多播组需要使用D类IP地址，其范围为224.0.0.0-239.255.255.255
    multicast_group.imr_interface.s_addr = INADDR_ANY;         // 加入多播组的主机地址信息
    setsockopt(recv_sockfd, IPPROTO_IP, IP_ADD_MEMBERSHIP, (void *) &multicast_addr, sizeof(multicast_addr));

    receiver_start();

    return 0;
}

