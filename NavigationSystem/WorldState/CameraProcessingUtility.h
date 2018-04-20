/****************************************************************************************
 *
 * File:
 *      CameraProcessingUtility.cpp
 *
 * Purpose:
 *      Utility functions for image acquisition and processing
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
#include <map>

#include "SystemServices/SysClock.h"
#include "SystemServices/Timer.h"
//#include "DataBase/DBHandler.h"
//#include "MessageBus/MessageTypes.h"
//#include "MessageBus/MessageBus.h"
//#include "MessageBus/ActiveNode.h"
#include "SystemServices/Logger.h"
//#include "WorldState/CollidableMgr/CollidableMgr.h"

using namespace std;
using namespace cv;

class CameraProcessingUtility {
public:
    CameraProcessingUtility(int cameraDeviceID, int detectorLoopTime, int maxCompassFrameTimerate);
    ~CameraProcessingUtility();

    void videoAcquisition(int cameraDeviceID);

    Mat getRoi();

    std::map<int16_t, int16_t> getRelDistances();
    
    //void processMessage(const Message* msg);

private:
    int freeSpaceProcessing(int cameraDeviceID);
    int computeRelDistances();

    Mat m_imgFullSize;
    Mat m_freeSpaceFrame;
    std::map<int16_t, int16_t> m_relBearingToRelObstacleDistance;
    /*
    const int lowFrameX;
    const int widthFrame;
    const int lowFrameY;
    const int heightFrame;
    */
    int m_cameraDeviceID;
    const int m_detectorLoopTime;
    const int m_maxCompassFrameTimeframe;

};
