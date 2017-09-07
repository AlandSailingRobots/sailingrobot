/****************************************************************************************
*
* File:
* 		CameraProcessingNode.cpp
*
* Purpose:
*     Receives frames from the camera and compass data from the CAN BUS, processes it and adds 
*     the obstacles found to the collidableMgr
*
* Developer Notes:
* NEW VERSION
*
***************************************************************************************/
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/photo.hpp>
#include <opencv2/videostab.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/video.hpp>

#include <vector>

#include "MessageBus/MessageTypes.h"
#include "MessageBus/MessageBus.h"
#include "MessageBus/ActiveNode.h"
#include "SystemServices/Logger.h"
#include "WorldState/CollidableMgr/CollidableMgr.h"
#include "Obstacle.hpp"
#include "CameraProcessingNode.hpp"

using namespace std;
using namespace cv;

/* @todo set those parameters from db */
#define CAMERA_DEVICE_ID 0 // 0 = default webcam
#define DETECTOR_LOOP_TIME 250 // in ms (250 * 20 ms = 5s)
#define MAX_COMPASS_FRAME_TIMEFRAME 10 // in ms
#define CAMERA_APERTURE_X 50
#define CAMERA_APERTURE_Y 50

vector<Obstacle> obstacle_list;
 
CameraProcessingNode::CameraProcessingNode(MessageBus& msgBus, CollidableMgr& collidableMgr)
  : ActiveNode(NodeID::CameraProcessingNode, msgBus), m_LoopTime(0.5), collidableMgr(collidableMgr) {
    msgBus.registerNode(*this, MessageType::CompassData);
  }

  CameraProcessingNode::~CameraProcessingNode() {
      
  }

  void CameraProcessingNode::processMessage(const Message* msg) {
    MessageType type = msg->messageType();
    switch (type) {
        // Set internal roll and heading values to the ones coming from compass
        case MessageType::CompassData :
            processCompassMessage((CompassDataMsg*) msg);
            break;
        default:
            return;
    }
 }

  void CameraProcessingNode::processCompassMessage(CompassDataMsg* msg) {
      m_compass_data.roll = msg->roll();
      m_compass_data.heading = msg->heading();
      m_compass_data.tmsp = SysClock::unixTime(); /** @todo Messages should have an internal timestamp, time not reliable due to possible glitches in message processing */
  }

  void CameraProcessingNode::addObstacleToCollidableMgr() {
    for (unsigned int i = 0; i < obstacle_list.size(); i++) 
    {
      this->collidableMgr->addVisualContact(obstacle_list[i].getId(), obstacle_list[i].getHeading());
    }
  }

  void CameraProcessingNode::start() {
    m_capture.open(CAMERA_DEVICE_ID); // Opens the camera handle
    if (m_capture.isOpened() == false) //  To check if object was associated to webcam successfully
    {
        Logger::error("Webcam not available");
        return;
    }
    else
    {
        // Start main thread
        runThread(CameraProcessingThreadFunc);
    }
  }
  
  bool CameraProcessingNode::init()
  {
      return true;
  }
  
  void CameraProcessingNode::detectObstacles()
  {
      Mat imgOriginal; // Input raw image
      Mat hsvImg;
      Mat threshImg;
      
      // Blob detection
      SimpleBlobDetector::Params params;
      // Change thresholds
      params.minThreshold = 10;
      params.maxThreshold = 200;
      // Filter by Area
      params.filterByArea = true;
      params.minArea = 300;
      // Filter by Circularity
      params.filterByCircularity = true;
      params.minCircularity = 0.3;
      // Filter by Color
      params.filterByColor = true;
      params.blobColor = 255;
      
#if CV_MAJOR_VERSION < 3   // For OpenCV 2
      // Set up detector with params
      SimpleBlobDetector detector(params);
#else
      // Set up detector with params
      Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);
#endif
      vector<KeyPoint> blobs;
      
      // Tilt correction
      Mat rot;
      Rect imageBox;
      
      // Set up frame size
      m_capture >> imgOriginal;
      Point2f center(imgOriginal.cols/2.0, imgOriginal.rows/2.0);
    
      // frame size
      double fWidth = m_capture.get(CV_CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
      //double fHeight = m_capture.get(CV_CAP_PROP_FRAME_HEIGHT); //get the height of frames of the videotracker->init(frame, bbox);
      float cameraAngleApertureXPerPixel = CAMERA_APERTURE_X/fWidth;
      //float cameraAngleApertureYPerPixel = CAMERA_APERTURE_Y/fHeight;
      
      for(int frame_n = 0; frame_n < DETECTOR_LOOP_TIME && m_capture.isOpened(); frame_n++) 
      {
        m_capture >> imgOriginal;

        if (imgOriginal.empty()) { // if frame read unsuccessfully
            Logger::error("video input frame not readable");
            break;
        }
          
        // Correct tilted image
        if(SysClock::unixTime() - m_compass_data.tmsp < MAX_COMPASS_FRAME_TIMEFRAME)
        {            
            // Create the rotation matrix
            rot = getRotationMatrix2D(center, m_compass_data.roll, 1.0);
            // Determine bounding rectangle
            imageBox = RotatedRect(center, imgOriginal.size(), m_compass_data.roll).boundingRect();
            // Adjust transformation matrix
            rot.at<double>(0,2) += imageBox.width/2.0 - center.x;
            rot.at<double>(1,2) += imageBox.height/2.0 - center.y;
            // Perform rotation
            warpAffine(imgOriginal, imgOriginal, rot, imageBox.size());
        }
        
        // Convert Original Image to HSV Thresh Image
        cvtColor(imgOriginal, hsvImg, CV_BGR2HSV); 
        
        // detect only the chosen color
        inRange(hsvImg, Scalar(0, 59, 222), Scalar(179, 166, 255), threshImg);
        
        // Apply filters
        GaussianBlur(threshImg, threshImg, Size(3, 3), 0);
        dilate(threshImg, threshImg, 0);
        erode(threshImg, threshImg, 0);
        
        // Init detector
        detector->detect( threshImg, blobs );
        
        // Adds a starting object to the list
        if(blobs.size() > 0 && obstacle_list.empty()) 
        {
            float heading = blobs[0].pt.x * cameraAngleApertureXPerPixel;
            Obstacle obs(frame_n, blobs[0].pt.x, blobs[0].pt.y, blobs[0].size, heading); // center coords and diameter
            obstacle_list.push_back(obs);
        }
        
        bool obstacle_present;        
        
        // Populate list of objects     
        for (unsigned int i = 0; i < blobs.size(); i++) 
        {     
            obstacle_present = false;
            
            for (unsigned int j = 0; j < obstacle_list.size(); j++) 
            {
                // If new detection, add to the list
                if( obstacle_list[j].compare(blobs[i].pt.x, blobs[i].pt.y) )
                {
                    obstacle_present = true;
                }
            }
            
            if(!obstacle_present)
            {
                // Create new obstacle
                float heading = blobs[i].pt.x * cameraAngleApertureXPerPixel;
                Obstacle obs(frame_n, blobs[i].pt.x, blobs[i].pt.y, blobs[i].size, heading); // center coords and diameter
                obstacle_list.push_back(obs);
            }
        }
        
        // Empty vectors
        blobs.clear();
        obstacle_list.clear();
        
        // Small delay
        waitKey(20);
      }
      
      // Empty obstacle list array
      obstacle_list.clear();
  }

  void CameraProcessingNode::CameraProcessingThreadFunc(ActiveNode* nodePtr) {
    CameraProcessingNode* node = dynamic_cast<CameraProcessingNode*> (nodePtr);

    Timer timer;
    timer.start();

    while(true) {
      node->detectObstacles();
      node->m_lock.lock();
      node->addObstacleToCollidableMgr();
      node->m_lock.unlock();
      timer.sleepUntil(node->m_LoopTime);
      timer.reset();
    }
  }
  
