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
#include <chrono>
#include <thread>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/photo.hpp>
#include <opencv2/videostab.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/video.hpp>

#include "Messages/CompassDataMsg.h"
#include "Messages/StateMessage.h"
#include "MessageBus/Message.h"
#include "DataBase/DBHandler.h"
#include "MessageBus/ActiveNode.h"
#include "MessageBus/MessageTypes.h"
#include "MessageBus/MessageBus.h"
#include "WorldState/CollidableMgr/CollidableMgr.h"
#include "SystemServices/Logger.h"
#include "SystemServices/Timer.h"

class CameraProcessingNode : public ActiveNode {
public:
  /*
  * Constructor, gets a pointer to messagebus and canservice
  */
  CameraProcessingNode(MessageBus& msgBus, DBHandler& dbhandler, CollidableMgr* collidableMgr);
  
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
  
  /*
   * Get the camera calibration data and correct the images retrieved
   */
  void applyCameraCorrection(std::string, cv::Mat&, cv::Mat&);

private:
    struct Compass
    {
        float roll;
        float heading;
        unsigned int tmsp; 
    } m_compass_data;

 /*
  * Update values from the database as the loop time of the thread and others parameters
  */
  void updateConfigsFromDB();
  
  /*
  * The function that thread works on
  */
  static void CameraProcessingThreadFunc(ActiveNode* nodePtr);

  /*
  * Private variables
  */
  double m_LoopTime; // in s
  int m_DetectorLoopTime; // in ms
  unsigned int m_TiltTimeDiffMax; // in ms (the max misalignment time between a frame and the tilt value from the compass)
  std::mutex m_lock;
  DBHandler& m_db;
  CollidableMgr* collidableMgr;
  cv::VideoCapture m_capture;
  
  const int CAMERA_DEVICE_ID = 0; // default webcam
  const int CAMERA_APERTURE_X = 320;
  const int CAMERA_APERTURE_Y = 240;
};
