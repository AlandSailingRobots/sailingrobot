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
*
*
***************************************************************************************/
#include "CameraProcessingNode.h"
#include <vector>

using namespace std;
using namespace cv;
using namespace chrono_literals;
 
CameraProcessingNode::CameraProcessingNode(MessageBus& msgBus, DBHandler& dbhandler, CollidableMgr* collidableMgr)
  : ActiveNode(NodeID::CameraProcessingNode, msgBus), m_LoopTime(1), m_DetectorLoopTime(250), m_TiltTimeDiffMax(10), m_db(dbhandler), collidableMgr(collidableMgr)
  {
    msgBus.registerNode(*this, MessageType::CompassData);
    msgBus.registerNode(*this, MessageType::ServerConfigsReceived);
  }

  CameraProcessingNode::~CameraProcessingNode() {
      
  }

  void CameraProcessingNode::updateConfigsFromDB(){
      m_LoopTime = m_db.retrieveCellAsDouble("config_camera_processing","1","loop_time");
      m_DetectorLoopTime = m_db.retrieveCellAsInt("config_camera_processing","250","detector_loop_time");
      m_TiltTimeDiffMax = m_db.retrieveCellAsInt("config_camera_processing","10","tilt_time_max_difference"); /** @todo find a better name */
  }
  
  void CameraProcessingNode::processMessage(const Message* msg) {
    MessageType type = msg->messageType();
    switch (type) {
        // Set internal roll and heading values to the ones coming from compass
        case MessageType::CompassData :
            processCompassMessage((CompassDataMsg*) msg);
            break;
        case MessageType::ServerConfigsReceived :
            updateConfigsFromDB();
            break;
        default:
            return;
    }
 }

  void CameraProcessingNode::processCompassMessage(CompassDataMsg* msg) {
      m_compass_data.roll = msg->roll();
      m_compass_data.heading = msg->heading();
      m_compass_data.tmsp = SysClock::unixTime(); /** @todo Messages should have an internal timestamp, received time not reliable due to possible glitches in message processing */
  }

  void CameraProcessingNode::start() {
      // Start main thread
    runThread(CameraProcessingThreadFunc);
  }
  
  bool CameraProcessingNode::init()
  {
      m_capture.open(CAMERA_DEVICE_ID); // Opens the camera handle
      if (m_capture.isOpened() == false) //  To check if object was associated to webcam successfully
      {
          Logger::error("Webcam not available");
          return false;
      }
      
      return true;
  }
  
  void CameraProcessingNode::detectObstacles()
  {
      Mat imgOriginal; // Input raw image
      Mat hsvImg; // HSV converted image
      // Image containers
      Mat frameGrayScale, roi, dst, cdst;

      // Noise removal kernel for the filters
      Mat kernel_ero = getStructuringElement(MORPH_RECT, Size(2,2));

      // Save detected lines
      std::vector<Vec4i> lines;
      std::vector<Point> locations;   // output, locations of non-zero pixels
      
      // For green dot detection
      std::vector<Vec3f> circles;

      // Tilt correction
      Mat rot;
      Rect imageBox;
      
      // Set up frame size
      m_capture >> imgOriginal;
      Point2f center(imgOriginal.cols/2.0, imgOriginal.rows/2.0);
    
      // frame size
      double fWidth = m_capture.get(CV_CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
      double fHeight = m_capture.get(CV_CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video
     // float cameraAngleApertureXPerPixel = CAMERA_APERTURE_X/fWidth;
     // float cameraAngleApertureYPerPixel = CAMERA_APERTURE_Y/fHeight;
      
      for(int frame_n = 0; frame_n < m_DetectorLoopTime && m_capture.isOpened(); frame_n++) 
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
         */
        // Check if compass data is up to date
        if(SysClock::unixTime() - m_compass_data.tmsp < m_TiltTimeDiffMax)
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
            std::this_thread::sleep_for(3s); 
            continue;
        }
        
        /*
         *-----------------------------------------------------------------
         * Denoising
         *-----------------------------------------------------------------
         */
        /** @todo Tune noise elimination filters parameters */
        //Convert image to grayscale
        cvtColor( imgOriginal, frameGrayScale, CV_BGR2GRAY );

        // Apply a Gaussian Filter to clear out general noise
        GaussianBlur( frameGrayScale, frameGrayScale, Size(7, 7), 2.0, 2.0 );

        // Remove spot noise
        medianBlur(frameGrayScale, frameGrayScale, 3);
        
        // Remove small objects
        erode( frameGrayScale, frameGrayScale, kernel_ero );
        dilate( frameGrayScale, frameGrayScale, kernel_ero );
        
        /*
         *-----------------------------------------------------------------
         * Find the horizon and set up the ROI (region of interest)
         * to the image surface beneath
         *-----------------------------------------------------------------
        */
        Canny(frameGrayScale, dst, 20, 50, 3);
        cvtColor(dst, cdst, COLOR_GRAY2BGR);

        HoughLinesP(dst, lines, 1, CV_PI/180, 50, 50, 10 );
        double theta1, theta2, hyp;

        // Horizon
        Vec4i max_l;
        double max_dist = -1.0;

        for( size_t i = 0; i < lines.size(); i++ )
        {
            Vec4i l = lines[i];
            theta1 = (l[3]-l[1]);
            theta2 = (l[2]-l[0]);
            hyp = hypot(theta1,theta2);

            Point p1, p2;
            p1=Point(l[0], l[1]);
            p2=Point(l[2], l[3]);

            //calculate angle in degrees
            float angle = atan2(p1.y - p2.y, p1.x - p2.x)*180/CV_PI;

            // +/- 45 deg max inclination and min 1/3 of the frame width size (tilt correction may not always work)
            if(angle < 135 || angle > 225 || hyp < fWidth/3)
            {
                lines.erase(lines.begin() + i);
                continue;
            }

            // select the greatest line only
            if (max_dist < hyp) 
            {
                max_l = l;
                max_dist = hyp;
            }
        }
        
         /*
         *-----------------------------------------------------------------
         * Define ROI (Region of Interest)
         *-----------------------------------------------------------------
         */
        Rect rect(Point(0, max_l[1]), Point(fWidth,fHeight));
        
        if( rect.area() > 0 )
        {
            roi = dst(rect);
        }
        // if the horizon was not found
        else
        {
            roi = dst;
        }
        
        /*
         *-----------------------------------------------------------------
         * Colour the frame in B/W according to lines detected
         *-----------------------------------------------------------------
         */
        for (int j = roi.cols-1; j>=0; j--) 
        {
            bool white = true;
            for (int i = roi.rows-1; i>=0; i--) 
            {
                 if (roi.at<unsigned char>(i,j) > 0)
                     white = false;
                 
                 if(white)
                     roi.at<unsigned char>(i,j)=255;
                 else
                     roi.at<unsigned char>(i,j)=0;
            }
        }
        
        /*
         *-----------------------------------------------------------------
         * Colour the frame in B/W according to lines detected
         *-----------------------------------------------------------------
         */
        for (int col = roi.cols-1; col>=0; col--) 
        {
            int row = roi.rows-1;
            do{
                row--;
            }while(roi.at<unsigned char>(row,col) != 255);
            
           // float bearing = col*webcamAngleApertureXPerPixel m_compass_data.heading;
            
           // collidableMgr->addVisualObstacle(row, bearing);
        }
        
        waitKey(10);
      }
  }

  void CameraProcessingNode::CameraProcessingThreadFunc(ActiveNode* nodePtr) {
    CameraProcessingNode* node = dynamic_cast<CameraProcessingNode*> (nodePtr);

    Timer timer;
    timer.start();

    while(true) {
      node->m_lock.lock();
      node->detectObstacles();
      node->m_lock.unlock();
      timer.sleepUntil(node->m_LoopTime);
      timer.reset();
    }
  }
  
  /** 
   *  @todo implement this function 
   * 
   *  Example use:
   *                // Load camera calibration data
   *                Mat frame, distCoeffs;
   *                std::string filename = "calibration.xml";
   *                applyCameraCorrection(filename, frame, distCoeffs);
   * 
   */
  void applyCameraCorrection(std::string filename, Mat& cameraMatrix2, Mat& distCoeffs2)
  {
    FileStorage fs2(filename, FileStorage::READ);

    fs2["camera_matrix"] >> cameraMatrix2;
    fs2["distortion_coefficients"] >> distCoeffs2;

    fs2.release();
  }
