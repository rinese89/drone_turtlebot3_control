#include "drone_pkg/NetworkServices.h"
#include <sys/types.h>
#include <sys/socket.h>

int NetworkServices::sendMessage(int curSocket, const char* message, int messageSize) {
    return send(curSocket, message, messageSize, 0);
}

int NetworkServices::receiveMessage(int curSocket, char* buffer, int bufSize) {
    return recv(curSocket, buffer, bufSize, 0);
}
