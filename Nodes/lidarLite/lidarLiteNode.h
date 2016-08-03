
/****************************************************************************************
 *
 * File:
 * 		lidarLiteNode.h
 *
 * Purpose:
 *		A lidarLiteNode which uses a lidar to measure the distance.
 * 		Will be used to find the boat-obstacle distance.
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/

#pragma once
#include "ActiveNode.h"
#include "lidarLite.h"
#include "Messages/LidarMsg.h"

class lidarLiteNode : public ActiveNode {
public:
    lidarLiteNode(MessageBus& msgBus,int delay,bool debug);//delay in microseconds at least 2000 or too much timeout
    lidarLiteNode(MessageBus& msgBus,int delay);//delay in microseconds

	///----------------------------------------------------------------------------------
	/// Initialises the connection with the lidar
	///
	///----------------------------------------------------------------------------------
	bool init();

	///----------------------------------------------------------------------------------
	/// Processes DataRequest messages.
	///
	///----------------------------------------------------------------------------------
	void processMessage(const Message* msgPtr);

	///----------------------------------------------------------------------------------
 	/// This function starts the  colorDetectionNode thread
 	///
 	///----------------------------------------------------------------------------------
	void start();
private:

	static void lidarThreadFunc(void* nodePtr);
    int 	m_delay;
    int		m_fd;
    bool 	m_debug;
    bool 	m_Initialised;

};
