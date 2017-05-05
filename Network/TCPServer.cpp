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

#include <sys/socket.h>
#include <arpa/inet.h> //inet_addr
#include <unistd.h>
#include <fcntl.h>
#include <sys/time.h>
#include <errno.h>
#include <cstring>

#include "SystemServices/Logger.h"


#define ERROR                   -1
#define SELECT_TIMEOUT_SEC      0
#define SELECT_TIMEOUT_USEC     500


///----------------------------------------------------------------------------------
TCPServer::TCPServer()
    : serverSocket(-1), highestFD(0)
{
    printf("%d\n", (int)clients.size());
}

///----------------------------------------------------------------------------------
int TCPServer::start( int port )
{
    int optionOn = 1;
    int rc = 0;
    sockaddr_in serverAddr;


    if( serverSocket > 0 )
    {
        Logger::warning( "Server is already running! %d", serverSocket );
        return ERROR;
    }

    serverSocket = socket( AF_INET, SOCK_STREAM, 0 );

    if( serverSocket < 0 ) 
    { 
        Logger::error( "Failed to create the server socket!" ); 
        return ERROR; 
    }

    rc = setsockopt( serverSocket, SOL_SOCKET, SO_REUSEADDR, (char*)&optionOn, sizeof(optionOn) );

    if( rc < 0 )
    {
        Logger::error( "Failed to change the server socket options!" ); 
        close( serverSocket );
        serverSocket = -1;
        return ERROR;
    }

    // Make the socket nonblocking, so we don't block when listening for new clients
    rc = fcntl( serverSocket, F_SETFL, fcntl(serverSocket, F_GETFL, 0) | O_NONBLOCK );
    
    if( rc < 0 ) 
    { 
        Logger::error( "Failed to make the server socket non-blocking!" ); 
        close( serverSocket );
        serverSocket = -1;
        return ERROR; 
    }

        // Bind the socket 
    memset( &serverAddr, 0, sizeof(serverAddr) );
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_addr.s_addr = htonl( INADDR_ANY );
    serverAddr.sin_port = htons( port );

    rc = bind( serverSocket, (sockaddr*)&serverAddr, sizeof(serverAddr) );

    if( rc < 0 )
    {
        Logger::error( "Failed to bind the server socket!" );
        close( serverSocket );
        serverSocket = -1;
        return ERROR;
    }

    // Make the socket listen for incoming connections 
    rc = listen( serverSocket, 32 );

    if( rc < 0 )
    {
        Logger::error( "Failed to make the server socket listen!" );
        close( serverSocket );
        serverSocket = -1;
        return ERROR;
    }

    FD_ZERO( &fdSet );
    FD_SET( serverSocket, &fdSet);
    highestFD = serverSocket;

    return 1;
}

///----------------------------------------------------------------------------------
void TCPServer::shutdown( )
{
    close( serverSocket );
    serverSocket = -1;

    // TODO: Disconnect clients
}

///----------------------------------------------------------------------------------
void TCPServer::broadcast( const uint8_t* data, const uint16_t size )
{
    int bytesLeft = size;
    int bytesSent = 0;
    int rc = 0;

    for( size_t i = 0; i < clients.size(); i++ )
    {
        tcpClient_t client = clients[i];
        while( bytesLeft > 0 )
        {
            rc = send( client.socketFD, &data[bytesSent], bytesLeft, MSG_NOSIGNAL );

            if( rc == -1 )
            {
                // Connection broke
                client.connected = false;
                break;
            }
            else
            {
                bytesLeft = bytesLeft - rc;
                bytesSent = bytesSent + rc;
            }
        }
    }
}

///----------------------------------------------------------------------------------
void TCPServer::acceptConnections()
{
    if( serverSocket > 0 )
    {
        int eventCount = 0;

        struct timeval timeout;
        timeout.tv_sec = SELECT_TIMEOUT_SEC;
        timeout.tv_usec = SELECT_TIMEOUT_USEC;

        // Make a copy
        fd_set tmp = fdSet;

        eventCount = select( highestFD + 1, &tmp, NULL, NULL, &timeout );

        if(eventCount == -1) 
        {
            Logger::error( "Failed to process server socket events!" );
        }

        // Check the listener socket, this would be a incoming connection.
        if( eventCount > 0 && FD_ISSET( serverSocket, &tmp ) )
        {
            int clientFD = 0;

            do {
                clientFD = accept( serverSocket, NULL, NULL );

                // Make the socket non-blocking
                if( clientFD > 0 )
                {
                    int rc = fcntl( clientFD, F_SETFL, fcntl(clientFD, F_GETFL, 0) | O_NONBLOCK );
                    if( rc < 0 )
                    {
                        Logger::error( "Failed to make the client socket blocking!" );
                        close ( clientFD );
                        clientFD = 0;
                    }
                }

                if( clientFD > 0 )
                {
                    Logger::info( "New client has connected!" );

                    tcpClient_t client;
                    client.socketFD = clientFD;
                    client.connected = true;
                    clients.push_back( client );
                }
            } while( clientFD != -1 );
        }
    }
}