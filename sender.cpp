#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define BUF_MAX_SIZE 1024

int udp_sender(int argc, char *argv[]) {
    int send_sockfd;                   // 发送方套接字描述符
    char buffer[BUF_MAX_SIZE];         // 数据缓冲区
    struct sockaddr_in multicast_addr; // 多播组套接字信息

    /* 参数检查 */
    if (argc != 3) {
        printf("Command format : %s <multicast IP> <port>!\n", argv[0]);
        exit(1);
    }

    /* 获取发送方套接字文件描述符 */
    if ((send_sockfd = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
        perror("socket error");
        exit(1);
    }

    /* 初始化多播组套接字信息 */
    memset(&multicast_addr, 0, sizeof(struct sockaddr_in));
    multicast_addr.sin_family = AF_INET;                 // 使用IPv4
    multicast_addr.sin_addr.s_addr = inet_addr(argv[1]); // 指定多播组IP
    multicast_addr.sin_port = htons(atoi(argv[2]));      // 对端口进行字节序转化

    /* 设定发送方发送的数据包的TTL值 */
    int ttl = 32;
    setsockopt(send_sockfd, IPPROTO_IP, IP_MULTICAST_TTL, (void *) &ttl, sizeof(ttl));

    /* 循环向多播组中发送消息 */
    while (1) {
        memset(buffer, 0, BUF_MAX_SIZE);
        printf("[sender] Input multicast message : ");
        scanf("%[^\n]%*c", buffer);
        sendto(send_sockfd, buffer, strlen(buffer), 0, (struct sockaddr *) &multicast_addr, sizeof(multicast_addr));
    }

    return 0;
}

int udp_send_init()
{
    int argc = 3;
    char *argv[] = {
        {""},
        {"224.0.0.2"},
        {"7033"},
    };

    udp_sender(argc, argv);

    return 0;
}
