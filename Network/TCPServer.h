/****************************************************************************************
 *
 * File:
 * 		TCPServer.h
 *
 * Purpose:
 *		
 *
 * License:
 *      This file is subject to the terms and conditions defined in the file 
 *      'LICENSE.txt', which is part of this source code package.
 *
 ***************************************************************************************/


#pragma once


#include <vector>
#include <sys/select.h>

#include <stdint.h>

struct tcpClient_t {
    int socketFD;
    bool connected;
};

class TCPServer {
public:
    ///----------------------------------------------------------------------------------
 	/// Constructs the TCP server.
 	///----------------------------------------------------------------------------------
    TCPServer();

    ///----------------------------------------------------------------------------------
 	/// Starts the TCP server.
 	///----------------------------------------------------------------------------------
    int start( int port );

    ///----------------------------------------------------------------------------------
 	/// Stops the TCP server
 	///----------------------------------------------------------------------------------
    void shutdown( );

    ///----------------------------------------------------------------------------------
 	/// Broadcasts data to all the connected clients. Not thread safe
 	///----------------------------------------------------------------------------------
    void broadcast( const uint8_t* data, const uint16_t size );

    ///----------------------------------------------------------------------------------
 	/// Checks for any inbound connections and accepts them. Not thread safe
 	///----------------------------------------------------------------------------------
    void acceptConnections();
private:
    int serverSocket;
    std::vector<tcpClient_t> clients;
    fd_set fdSet;
    int highestFD;
};