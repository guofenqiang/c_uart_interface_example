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

class Sender
{
public:
    Sender(Serial_Port *_port);
    ~Sender();
    int udp_send_init();
    int send_start();
    int udp_sender();


private:
    int sockfd;
    struct sockaddr_in multicast_addr;
    struct in_addr local_if;
    struct ip_mreqn mreq;

    Serial_Port *port;
};

void *udp_sender_loop(void *args);

#endif // SENDER_H_