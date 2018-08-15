
/****************************************************************************************
 *
 * File:
 * 		LidarLiteNode.h
 *
 * Purpose:
 *		A LidarLiteNode which uses a lidar to measure the distance.
 * 		Will be used to find the boat-obstacle distance.
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/

#pragma once
#include "../MessageBus/ActiveNode.h"
#include "../Messages/LidarMsg.h"
#include "lidarLite/lidarLite.h"

class LidarLiteNode : public ActiveNode {
   public:
    LidarLiteNode(MessageBus& msgBus,
                  int delay,
                  bool debug);  // delay in msat least 2ms or too much timeout
    LidarLiteNode(MessageBus& msgBus, int delay);

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
    static void LidarThreadFunc(ActiveNode* nodePtr);
    int m_delay;  // units : milliseconds
    int m_fd;
    bool m_debug;
    bool m_Initialised;
};
