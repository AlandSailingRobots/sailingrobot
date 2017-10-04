/****************************************************************************************
*
* File:
* 		ObstacleDetectionTest.cpp
*
* Purpose:
*		Integration test for obstacle detection with the camera.
*
*
* Developer Notes:
*
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

#include "SystemServices/SysClock.h"
#include "SystemServices/Timer.h"
#include "DataBase/DBHandler.h"
#include "MessageBus/MessageTypes.h"
#include "MessageBus/MessageBus.h"
#include "MessageBus/ActiveNode.h"
#include "SystemServices/Logger.h"
#include "WorldState/CollidableMgr/CollidableMgr.h"
#include "WorldState/obstacledetection/Obstacle.hpp"

using namespace std;
using namespace cv;

#define CAMERA_DEVICE_ID 0 // 0 = default webcam
#define DETECTOR_LOOP_TIME 250 // in ms (250 * 20 ms = 5s)
#define MAX_COMPASS_FRAME_TIMEFRAME 10 // in ms
#define CAMERA_APERTURE_X 50
#define CAMERA_APERTURE_Y 50

DBHandler dbHandler("../asr.db");
MessageBus msgBus;
CollidableMgr cMgr;
vector<Obstacle> obstaclelist;
bool recordVid = true;

void messageLoop() {
    msgBus.run();
}

int main() 
{
  Logger::init("ObstacleDetectionTest.log");

  cMgr.startGC();

  std::thread thr(messageLoop);
  thr.detach();
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
    
  VideoCapture m_capture(0); // Opens the camera handle
  if (m_capture.isOpened() == false) //  To check if object was associated to webcam successfully
  {
      Logger::error("Webcam not available");
      return -1;
  } 
  
  double fWidth = m_capture.get(CV_CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
  double fHeight = m_capture.get(CV_CAP_PROP_FRAME_HEIGHT); //get the height of frames of the videotracker->init(frame, bbox);
  Size frameSize(static_cast<int>(fWidth), static_cast<int>(fHeight));
  VideoWriter outputVideo ("video.avi", CV_FOURCC('D','I','V','X'), 20, frameSize, true);

  if (!outputVideo.isOpened())
  {
    Logger::error("Could not open the output video for write");
    recordVid = false;
  }
    
    if(recordVid == false)
        remove("video.avi");
    
      Mat imgOriginal; // Input raw image
      Mat hsvImg; // HSV converted image
      Mat threshImg; // Filtered HSV image
      
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
      
      // Set up detector with params
      Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);

      vector<KeyPoint> blobs;
      
      // For green dot detection
      vector<Vec3f> circles;

      // Tilt correction
      Mat rot;
      Rect imageBox;
      
      // Set up frame size
      m_capture >> imgOriginal;
      Point2f center(imgOriginal.cols/2.0, imgOriginal.rows/2.0);
    
      // frame size
      float cameraAngleApertureXPerPixel = CAMERA_APERTURE_X/fWidth;
      //float cameraAngleApertureYPerPixel = CAMERA_APERTURE_Y/fHeight;
      
      for(int frame_n = 0; frame_n < DETECTOR_LOOP_TIME && m_capture.isOpened(); frame_n++) 
      {
        m_capture >> imgOriginal;

        if (imgOriginal.empty()) { // if frame read unsuccessfully
            Logger::error("video input frame not readable");
            break;
        }
          
        /*
         *-----------------------------------------------------------------
         * Correct tilted image
         *-----------------------------------------------------------------
         *
        // Check if compass data is up to date
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
        */
        /*
         *-----------------------------------------------------------------
         * Convert image to HSV color space
         *-----------------------------------------------------------------
         */
        // Convert Original Image to HSV Thresh Image
        cvtColor(imgOriginal, hsvImg, CV_BGR2HSV); 
        
        /* 
         *-----------------------------------------------------------------
         * Find green circle indicating camera recalibration
         *-----------------------------------------------------------------
         */
#define WEBCAM
#ifndef WEBCAM
        // Find green pixels
        inRange(hsvImg, Scalar(45, 100, 100), Scalar(75, 255, 255), hsvImg);

        // Avoid false positives with a general blur
        GaussianBlur(hsvImg, hsvImg, Size(9, 9), 2, 2);

        // Find circles
        HoughCircles(hsvImg, circles, CV_HOUGH_GRADIENT, 1, hsvImg.rows/8, 85, 15, 0, 0);

        if(circles.size() != 0) 
        {
            // A green circle has been detected, he thermal camera is recalibrating and frames may be unreliable and corrupted, sleep thread
            Logger::info("Camera is recalibrating, thread will sleep for 3 seconds");
            Timer t;
            t.start();
            t.sleepUntil(3);
            t.reset();
            continue;
        }
#endif
        
        /*
         *-----------------------------------------------------------------
         * Denoising
         *-----------------------------------------------------------------
         */
        // Apply filters
        GaussianBlur(threshImg, threshImg, Size(3, 3), 0);
        medianBlur(threshImg, threshImg, 3);
        dilate(threshImg, threshImg, 0);
        erode(threshImg, threshImg, 0);
        
        /*
         *-----------------------------------------------------------------
         * Find blobs
         *-----------------------------------------------------------------
         */
        // Init detector
        detector->detect( threshImg, blobs );
        
        // Adds a starting object to the list
        if(blobs.size() > 0 && obstaclelist.empty()) 
        {
            float heading = blobs[0].pt.x * cameraAngleApertureXPerPixel;
            Obstacle obs(frame_n, blobs[0].pt.x, blobs[0].pt.y, blobs[0].size, heading); // center coords and diameter
            obstaclelist.push_back(obs);
        }
        
        bool obstacle_present;        
        
        // Populate list of objects     
        for (unsigned int i = 0; i < blobs.size(); i++) 
        {     
            obstacle_present = false;
            
            for (unsigned int j = 0; j < obstaclelist.size(); j++) 
            {
                // If new detection, add to the list
                if( obstaclelist[j].compare(blobs[i].pt.x, blobs[i].pt.y) )
                {
                    obstacle_present = true;
                }
            }
            
            if(!obstacle_present)
            {
                // Create new obstacle
                float heading = blobs[i].pt.x * cameraAngleApertureXPerPixel;
                Obstacle obs(frame_n, blobs[i].pt.x, blobs[i].pt.y, blobs[i].size, heading); // center coords and diameter
                obstaclelist.push_back(obs);
                Logger::info("new obstacle found");
            }
        }
        
        // Empty vectors
        blobs.clear();
        obstaclelist.clear();
        
        // Small delay
        waitKey(20);
        
        if(recordVid)
            outputVideo.write(imgOriginal);
      }
  }
