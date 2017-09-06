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
#include "DataBase/DBHandler.h"
#include "MessageBus/MessageTypes.h"
#include "MessageBus/MessageBus.h"
#include "MessageBus/ActiveNode.h"
#include "SystemServices/Logger.h"
#include "WorldState/CollidableMgr/CollidableMgr.h"
#include "WorldState/obstacledetection/Obstacle.hpp"
#include "WorldState/obstacledetection/CameraProcessingNode.hpp"

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
    
  VideoCapture capture(0); // Opens the camera handle
  if (capture.isOpened() == false) //  To check if object was associated to webcam successfully
  {
      Logger::error("Webcam not available");
      return -1;
  } 

	Mat imgOriginal;		// Input image
	Mat hsvImg;				// HSV Image
	Mat threshImg;			// Thresh Image

    Mat im_with_blobs;
	
	 // Blob detection
      SimpleBlobDetector::Params params;
      // Change thresholds
      params.minThreshold = 10;
      params.maxThreshold = 500;
      // Filter by Area
      params.filterByArea = true;
      params.minArea = 100;
      // Filter by Circularity
      params.filterByCircularity = true;
      params.minCircularity = 0.2;
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
      capture >> imgOriginal;
      Point2f center(imgOriginal.cols/2.0, imgOriginal.rows/2.0);
    
      // frame size
      double fWidth = capture.get(CV_CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
      //double fHeight = capture.get(CV_CAP_PROP_FRAME_HEIGHT); //get the height of frames of the videotracker->init(frame, bbox);
      float cameraAngleApertureXPerPixel = CAMERA_APERTURE_X/fWidth;
      //float cameraAngleApertureYPerPixel = CAMERA_APERTURE_Y/fHeight;
      
  while (true) {
    //std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    Logger::info("new cycle");
    
    for(int frame_n = 0; frame_n < DETECTOR_LOOP_TIME && capture.isOpened(); frame_n++) 
    {
        capture >> imgOriginal;

        if (imgOriginal.empty()) { // if frame read unsuccessfully
            Logger::error("video input frame not readable");
            break;
        }
        
        namedWindow("Raw Video", CV_WINDOW_AUTOSIZE);
        imshow("Raw Video", imgOriginal);
        
        waitKey(10);
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
        if(blobs.size() > 0 && obstaclelist.empty()) 
        {
            Logger::info("Obstacles detected but obstacle list empty");
            
            float bearing = blobs[0].pt.x * cameraAngleApertureXPerPixel;
            Obstacle obs(frame_n, blobs[0].pt.x, blobs[0].pt.y, blobs[0].size, bearing); // center coords and diameter
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
                    cout<<"already seen obstacle n. "<<obstaclelist[j]getId()<<endl;
                }
            }
            
            if(!obstacle_present)
            {
                // Create new obstacle
                float heading = blobs[i].pt.x * cameraAngleApertureXPerPixel;
                Obstacle obs(frame_n, blobs[i].pt.x, blobs[i].pt.y, blobs[i].size, heading); // center coords and diameter
                obstaclelist.push_back(obs);
                cout<<"hey there obstacle n. "<<obs.getId()<<endl;
            }
            
            // Remove it afterwards
            blobs.erase(blobs.begin()+i);
        }

        drawKeypoints( threshImg, blobs, im_with_blobs, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );

        namedWindow("Blobs", CV_WINDOW_AUTOSIZE);
        imshow( "Blobs" , im_with_blobs );
        
        // Empty blobs array
        blobs.clear();
        
        // Small delay
        waitKey(20);
      }
      
      for (unsigned int i = 0; i < obstaclelist.size(); i++) 
      {
          Logger::info("Obstacle added to the CollidableMgr");
          cMgr.addVisualContact(obstaclelist[i].getId(), obstaclelist[i].getHeading());
      }
      
      // Empty obstacle list array
      obstaclelist.clear();
  }
}
          
          
    // Logger::info("Collidable manager size: " + std::to_string(cMgr.getAISContacts().length()));
    // auto colList = cMgr.getAISContacts();
    // for (int i = 0; i<cMgr.getAISContacts().length(); i++) {
    //     auto t = colList.next();
    //     Logger::info("MMSI: " + std::to_string(t.mmsi) + ", Lat: " + std::to_string(t.latitude) + ", Lon: " + std::to_string(t.longitude) +
    //             ", COG: " + std::to_string(t.course) + " (" + std::to_string(t.course*180/3.141592) + ")" + ", SOG: " + std::to_string(t.speed) +
    //             " (" + std::to_string(t.speed*1.9438) + ")" + ", Length: " + std::to_string(t.length) + ", Beam: " + std::to_string(t.beam) +
    //             ", Report age: " + std::to_string(now-t.lastUpdated));
    // }
