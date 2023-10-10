#ifndef SENDER_H_
#define SENDER_H_

#include "serial_port.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <net/if.h>
#include "uav_interface.h"

class Sender
{
public:
    Sender(UAV_Interface *uav_interface_);
    ~Sender();
    int udp_send_init();
    int send_start();
    int udp_sender();


private:
    int sockfd;
    struct sockaddr_in multicast_addr;
    struct in_addr local_if;
    struct ip_mreqn mreq;

    UAV_Interface *_uav_interface;
};

void *udp_sender_loop(void *args);

#endif // SENDER_H_