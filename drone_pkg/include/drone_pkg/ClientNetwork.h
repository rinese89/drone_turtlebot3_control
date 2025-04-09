#ifndef CLIENTNETWORK_H
#define CLIENTNETWORK_H

#include "drone_pkg/NetworkServices.h"

#include <iostream>
#include <cstring>      // Para memset
#include <cstdlib>      // Para exit()
#include <unistd.h>     // Para close()
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <errno.h>
#include <stdio.h> 

// size of our buffer
#define MAX_PACKET_SIZE 1000000
#define DEFAULT_BUFLEN 1024
#define TEL_PACKET_SIZE 56
#define TEL_PACKET_CONTROL 27

// port to connect sockets through 
#define HOST "192.168.0.102"
#define PORT_CMD "8000"
#define PORT_TEL "9007"

//#define HOST "127.0.0.1"
//#define PORT_CMD "12345"
//#define PORT_TEL "12345"



class ClientNetwork
{

public:

    // socket for client to connect to server
    int ConnectSocket;

    // ctor/dtor
    ClientNetwork(char*, char*);
    ~ClientNetwork(void);

    int receivePacket(char*, int);
    void sendPacket(char*, int);

};

#endif // CLIENTNETWORK_H