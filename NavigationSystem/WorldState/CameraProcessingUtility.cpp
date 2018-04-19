/****************************************************************************************
 *
 * File:
 * 		CameraProcessingUtility.cpp
 *
 * Purpose:
 *		Utility functions for image acquisition and processing
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
#include <thread>

#include "SystemServices/SysClock.h"
#include "SystemServices/Timer.h"
//#include "DataBase/DBHandler.h"
//#include "MessageBus/MessageTypes.h"
//#include "MessageBus/MessageBus.h"
//#include "MessageBus/ActiveNode.h"
#include "SystemServices/Logger.h"
//#include "WorldState/CollidableMgr/CollidableMgr.h"
 #include "CameraProcessingUtility.h"

using namespace std;
using namespace cv;

#define CAMERA_DEVICE_ID 0 // 0 = default webcam
#define DETECTOR_LOOP_TIME 250 // in ms (250 * 20 ms = 5s)
#define MAX_COMPASS_FRAME_TIMEFRAME 10 // in ms

CameraProcessingUtility::CameraProcessingUtility(int cameraDeviceID, int detectorLoopTime, int maxCompassFrameTimerate)
  :  CAMERA_DEVICE_ID, DETECTOR_LOOP_TIME, MAX_COMPASS_FRAME_TIMEFRAME {
		// these are retrieved by trial and error, to get the roi containing the thermal imager data
		const int lowFrameX = 68; //29
		const int widthFrame = 585; //257
		const int lowFrameY = 98; //30
		const int heightFrame = 381;  //195
  } 

CameraProcessingUtility::~CameraProcessingUtility() {

}





//DBHandler dbHandler("../asr.db");
//MessageBus msgBus;
//CollidableMgr cMgr;
struct Compass
{
    float roll = 0;
    float heading = 0;
    int tmsp = 0;
} m_compass_data;

//void messageLoop() {
//    msgBus.run();
//}

Mat dilateReconstruction(Mat imgReference, Mat imgMarker, Mat kernel)
{
    Mat imgRec, imgDil, imgResult;
    imgRec = imgMarker.clone();   
    while (cv::countNonZero(imgRec != imgResult) != 0)  // if the two matrices are equal, we end the loop
    {
        imgResult = imgRec.clone();
        dilate(imgResult, imgDil, kernel);
        imgRec = min(imgDil, imgReference);

    }
    return imgResult;
}

