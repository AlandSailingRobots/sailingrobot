/****************************************************************************************
 *
 * File:
 * 		TCPServer.h
 *
 * Purpose:
 *
 * Packet Information:
 *      The TCP server expects its data in a packet like format. Each of this packets 
 *      must start with a packet length(2 bytes, unsigned short) followed by the data. 
 *      Packet data cannot be over 512 bytes. This was a limit imposed by the original 
 *      author.
 *		
 * Developer Notes:
 *      * All timeouts are in seconds
 *
 *      * Most of the issues related to thread safely are due to clients connecting. If 
 *        no clients are expected to connect or disconenct during most of these functions
 *        then there won't be a problem. However always double check the function
 *
 *      * Currently this does not really handle clients disconnecting nicely 
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

struct TCPPacket_t {
    int         socketFD; // Either the destination or sender, depends on context
    uint16_t    length; // Data length, max 512
    uint8_t     data[512];
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

    ///----------------------------------------------------------------------------------
 	/// Blocks until there is an inbound connection, or times out. If there is a 
    /// connection it is accepted and this function returns 1, otherwise 0 is returned. 
    /// A timeout of 0 means a indefinite block. Not thread safe
 	///----------------------------------------------------------------------------------
    int acceptConnection( uint32_t timeout );

    ///----------------------------------------------------------------------------------
 	/// Attempts to read a incoming packet from any connected client. Returns 1 if the 
    /// read was succesful, or 0 if the read timed out. A timeout of 0 means the function
    /// blocks until a packet has been succesfully received. Not thread safe
 	///----------------------------------------------------------------------------------
    int readPacket( TCPPacket_t& packet, uint32_t timeout );

    ///----------------------------------------------------------------------------------
 	/// Clears a socket's read buffer
 	///----------------------------------------------------------------------------------
    void clearSocketBuffer( int socketFD );

    ///----------------------------------------------------------------------------------
 	/// Sends data to a socket.
 	///----------------------------------------------------------------------------------
    int sendData( int socketFD, void* data, uint16_t size );
private:
    ///----------------------------------------------------------------------------------
 	/// Setups a new client, and begins tracking it.
 	///----------------------------------------------------------------------------------
    int setupClient( int clientFD );

    int serverSocket;
    std::vector<tcpClient_t> clients;
    fd_set fdSet;
    int highestFD;
};