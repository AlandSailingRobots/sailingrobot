/****************************************************************************************
 *
 * File:
 * 		TCPServer.cpp
 *
 * Purpose:
 *
 *
 * License:
 *      This file is subject to the terms and conditions defined in the file
 *      'LICENSE.txt', which is part of this source code package.
 *
 ***************************************************************************************/

#include "TCPServer.h"

#include <arpa/inet.h>  //inet_addr
#include <errno.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>
#include <cstring>

#include "SystemServices/Logger.h"
#include "SystemServices/SysClock.h"

#define ERROR -1
#define SELECT_TIMEOUT_SEC 0
#define SELECT_TIMEOUT_USEC 500

///----------------------------------------------------------------------------------
TCPServer::TCPServer() : serverSocket(-1), highestFD(0) {
    printf("%d\n", (int)clients.size());
}

///----------------------------------------------------------------------------------
int TCPServer::start(int port) {
    int optionOn = 1;
    int rc = 0;
    sockaddr_in serverAddr;

    if (serverSocket > 0) {
        Logger::warning("Server is already running! %d", serverSocket);
        return ERROR;
    }

    serverSocket = socket(AF_INET, SOCK_STREAM, 0);

    if (serverSocket < 0) {
        Logger::error("Failed to create the server socket!");
        return ERROR;
    }

    rc = setsockopt(serverSocket, SOL_SOCKET, SO_REUSEADDR, (char*)&optionOn, sizeof(optionOn));

    if (rc < 0) {
        Logger::error("Failed to change the server socket options!");
        close(serverSocket);
        serverSocket = -1;
        return ERROR;
    }

    // Make the socket nonblocking, so we don't block when listening for new
    // clients
    rc = fcntl(serverSocket, F_SETFL, fcntl(serverSocket, F_GETFL, 0) | O_NONBLOCK);

    if (rc < 0) {
        Logger::error("Failed to make the server socket non-blocking!");
        close(serverSocket);
        serverSocket = -1;
        return ERROR;
    }

    // Bind the socket
    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    serverAddr.sin_port = htons(port);

    rc = bind(serverSocket, (sockaddr*)&serverAddr, sizeof(serverAddr));

    if (rc < 0) {
        Logger::error("Failed to bind the server socket!");
        close(serverSocket);
        serverSocket = -1;
        return ERROR;
    }

    // Make the socket listen for incoming connections
    rc = listen(serverSocket, 32);

    if (rc < 0) {
        Logger::error("Failed to make the server socket listen!");
        close(serverSocket);
        serverSocket = -1;
        return ERROR;
    }

    FD_ZERO(&fdSet);
    FD_SET(serverSocket, &fdSet);
    highestFD = serverSocket;

    return 1;
}

///----------------------------------------------------------------------------------
void TCPServer::shutdown() {
    close(serverSocket);
    serverSocket = -1;

    // TODO: Disconnect clients
}

///----------------------------------------------------------------------------------
void TCPServer::broadcast(const uint8_t* data, const uint16_t size) {
    int bytesLeft = size;
    int bytesSent = 0;
    int rc = 0;

    for (size_t i = 0; i < clients.size(); i++) {
        tcpClient_t client = clients[i];
        while (bytesLeft > 0) {
            rc = send(client.socketFD, &data[bytesSent], bytesLeft, MSG_NOSIGNAL);

            if (rc == -1) {
                // Connection broke
                client.connected = false;
                break;
            } else {
                bytesLeft = bytesLeft - rc;
                bytesSent = bytesSent + rc;
            }
        }
    }
}

///----------------------------------------------------------------------------------
void TCPServer::acceptConnections() {
    if (serverSocket > 0) {
        int eventCount = 0;

        struct timeval timeout;
        timeout.tv_sec = SELECT_TIMEOUT_SEC;
        timeout.tv_usec = SELECT_TIMEOUT_USEC;

        // Make a copy
        fd_set tmp = fdSet;

        eventCount = select(highestFD + 1, &tmp, NULL, NULL, &timeout);

        if (eventCount == -1) {
            Logger::error("Failed to process server socket events!");
        }

        // Check the listener socket, this would be a incoming connection.
        if (eventCount > 0 && FD_ISSET(serverSocket, &tmp)) {
            int clientFD = 0;
            int rc = 0;

            do {
                clientFD = accept(serverSocket, NULL, NULL);
                rc = setupClient(clientFD);

            } while (rc <= 0);
        }
    }
}

///----------------------------------------------------------------------------------
int TCPServer::acceptConnection(uint32_t timeout) {
    if (serverSocket > 0) {
        bool timedOut = false;
        int clientFD = 0;
        int startTime = SysClock::unixTime();
        unsigned long endTime = startTime + timeout;

        while (!timedOut) {
            clientFD = accept(serverSocket, NULL, NULL);

            if (clientFD > 0 && setupClient(clientFD)) {
                return 1;
            }

            if (timeout > 0 && (SysClock::unixTime() > endTime)) {
                timedOut = false;
            }
        }
    }

    return 0;
}

///----------------------------------------------------------------------------------
int TCPServer::readPacket(TCPPacket_t& packet, uint32_t timeout) {
    if (serverSocket > 0) {
        // Makes things a little more thread safe, but still not truely thread safe,
        // our weakness being this copy
        std::vector<tcpClient_t> clientCopy = clients;

        int bytesRead = 0;
        uint16_t length = 0;
        //bool timedOut = false;
        //int startTime = SysClock::unixTime();
        //unsigned long endTime = startTime + timeout;

        //while (!timedOut) {

            for (uint16_t i = 0; i < clientCopy.size(); i++) {
                tcpClient_t* client = &clientCopy[i];
                bytesRead = read(client->socketFD, &length, 2);

                // We have a packet read for us
                if (bytesRead == 2) {
                    bytesRead = read(client->socketFD, packet.data, length);

                    // Valid packet
                    if (bytesRead == length) {
                        packet.socketFD = client->socketFD;
                        packet.length = length;

                        return 1;
                    } else {
                        Logger::info("invalid packet, length: %i", length);
                        Logger::info("invalid packet, bytes read: %i", bytesRead);
                    }
                }
            }

            //if ((timeout > 0 )&& (SysClock::unixTime() > endTime)) {
                //Logger::info("TCP BROKE");
                //timedOut = true;
            //}
        //}
    }
    return 0;
}

///----------------------------------------------------------------------------------
void TCPServer::clearSocketBuffer(int socketFD) {
    static uint8_t buffer[512];

    while (read(socketFD, buffer, 512) > 0)
        ;
}

///----------------------------------------------------------------------------------
int TCPServer::sendData(int socketFD, void* data, uint16_t size) {
    if (socketFD > 0) {
        int dataSent = 0;

        // Send the length first
        dataSent = write(socketFD, &size, 2);

        // The otherside has probably disconnected
        if (dataSent != 2) {
            return 0;
        }

        // Send the data second
        dataSent = write(socketFD, data, size);

        // The otherside has probably disconnected
        if (dataSent != size) {
            return 0;
        }
    }

    return 0;
}

///----------------------------------------------------------------------------------
int TCPServer::setupClient(int clientFD) {
    // Make the socket non-blocking
    if (clientFD > 0) {
        int rc = fcntl(clientFD, F_SETFL, fcntl(clientFD, F_GETFL, 0) | O_NONBLOCK);
        if (rc < 0) {
            Logger::error("Failed to make the client socket blocking!");
            close(clientFD);
            return 0;
        }
    }

    Logger::info("New client has connected!");

    tcpClient_t client;
    client.socketFD = clientFD;
    client.connected = true;
    clients.push_back(client);
    return 1;
}
