/****************************************************************************************
*
* File:
* 		CameraProcessingNode.h
*
* Purpose:
*     Receives compass data from the CAN BUS and processes frames coming from the camera
*     sending detected obstacles' bearing to the collidableMgr
*
* Developer Notes:
*
*
***************************************************************************************/

#pragma once

#include <mutex>
#include <vector>
#include <chrono>
#include <thread>

#include "Messages/CompassDataMsg.h"
#include "Messages/StateMessage.h"
#include "MessageBus/Message.h"
#include "MessageBus/ActiveNode.h"
#include "MessageBus/MessageTypes.h"
#include "MessageBus/MessageBus.h"
#include "WorldState/CollidableMgr/CollidableMgr.h"
#include "SystemServices/Logger.h"
#include "SystemServices/Timer.h"

#include "Obstacle.hpp"

class CameraProcessingNode : public ActiveNode {
public:
  /*
  * Constructor, gets a pointer to messagebus and canservice
  */
  CameraProcessingNode(MessageBus& msgBus, CollidableMgr& collidableMgr);
  
  /*
   *  Empty destructor
   */
  ~CameraProcessingNode();

  /*
  * Processes messages received
  */
  void processMessage(const Message* msg);
  
  /*
   * Process compass message
   */
  void processCompassMessage(CompassDataMsg* msg);
  
  /*
   *  Detect obstacles in the given set of frames
   */
  void detectObstacles();
  
  /*
  * Starts the worker thread
  */
  void start();
  
  /*
  * Initialisation function
  */
  bool init();

private:
    struct Compass
    {
        float roll;
        float heading;
        int tmsp; 
    } m_compass_data;
  /*
  * Sends the data to the collidable manager
  */
  void addObstacleToCollidableMgr();

  /*
  * The function that thread works on
  */
  static void CameraProcessingThreadFunc(ActiveNode* nodePtr);

  /*
  * Private variables
  */
  //Compass m_compass_data;
  double m_LoopTime;
  std::mutex m_lock;
  CollidableMgr* collidableMgr;
  cv::VideoCapture m_capture;
};
