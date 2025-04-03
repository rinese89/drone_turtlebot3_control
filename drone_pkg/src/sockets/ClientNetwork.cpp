#include <iostream>
#include <cstring>      // Para memset
#include <cstdlib>      // Para exit()
#include <unistd.h>     // Para close()
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <errno.h>

#include "drone_pkg/ClientNetwork.h"

ClientNetwork::ClientNetwork(char* host, char* port) {
    struct addrinfo hints, *result, *ptr;
    
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_INET;       // IPv4
    hints.ai_socktype = SOCK_STREAM; // TCP
    hints.ai_protocol = IPPROTO_TCP; // Protocolo TCP

    // Obtener dirección del servidor
    int iResult = getaddrinfo(host, port, &hints, &result);
    if (iResult != 0) {
        std::cerr << "getaddrinfo error: " << gai_strerror(iResult) << std::endl;
        exit(1);
    }

    // Intentar conectarse a alguna direccion obtenida
    for (ptr = result; ptr != nullptr; ptr = ptr->ai_next) {
        // Crear socket
        ConnectSocket = socket(ptr->ai_family, ptr->ai_socktype, ptr->ai_protocol);
        if (ConnectSocket == -1) {
            perror("Error al crear socket");
            continue;
        }

        // Intentar conectarse
        if (connect(ConnectSocket, ptr->ai_addr, ptr->ai_addrlen) == -1) {
            perror("Error al conectar");
            close(ConnectSocket);
            ConnectSocket = -1;
            continue;
        }
        break;
    }

    freeaddrinfo(result);

    if (ConnectSocket == -1) {
        std::cerr << "No se pudo conectar al servidor" << std::endl;
        exit(1);
    }

    std::cout << "OK. Conectado\n";
}

ClientNetwork::~ClientNetwork() {
    if (ConnectSocket != -1) {
        close(ConnectSocket);
    }
}

int ClientNetwork::receivePacket(char* recvbuf, int bufsize) {
    int iResult = recv(ConnectSocket, recvbuf, bufsize, 0);
    if (iResult == 0) {
        std::cerr << "Conexión cerrada por el servidor" << std::endl;
        close(ConnectSocket);
        exit(1);
    } else if (iResult == -1) {
        perror("Error en recv");
        close(ConnectSocket);
        exit(1);
    }
    return iResult;
}

void ClientNetwork::sendPacket(char* packets, int totalSize) {
    int iSendResult = send(ConnectSocket, packets, totalSize, 0);
    if (iSendResult == -1) {
        perror("Error en send");
        close(ConnectSocket);
        exit(1);
    }
    std::cout << "Enviado: " << packets << std::endl;
}
