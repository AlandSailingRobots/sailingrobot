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
 *      Need to delete all the unneccessary include here
 *
 ***************************************************************************************/
#pragma once

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>


#include <vector>
#include <thread>
#include <mutex>
#include <map>

#include "SystemServices/SysClock.h"
#include "SystemServices/Timer.h"
#include "DataBase/DBHandler.h"
#include "Messages/CompassDataMsg.h"
#include "MessageBus/MessageTypes.h"
#include "MessageBus/MessageBus.h"
#include "MessageBus/ActiveNode.h"
#include "SystemServices/Logger.h"
#include "WorldState/CollidableMgr/CollidableMgr.h"


class CameraProcessingUtility : public ActiveNode {
public:
    CameraProcessingUtility(MessageBus& msgBus, DBHandler& dbhandler, CollidableMgr* collidableMgr);
    ~CameraProcessingUtility();

    bool init();
    void start();
    void stop();

    cv::Mat getRoi();

    std::map<int16_t, uint16_t> getRelDistances();
    
    void processMessage(const Message* msg);

private:
    void freeSpaceProcessing();
    int computeRelDistances();
    void addCameraDataToCollidableMgr();
    static void CameraProcessingUtilityThreadFunc(ActiveNode* nodePtr);
    void processCompassMessage(const CompassDataMsg* msg);

    cv::VideoCapture m_capture;
    cv::Mat m_imgFullSize;
    cv::Mat m_freeSpaceFrame;
    std::map<int16_t, uint16_t> m_relBearingToRelObstacleDistance;

    int m_cameraDeviceID;
    const int m_detectorLoopTime;
    const int m_maxCompassFrameTimeframe;
    CollidableMgr* collidableMgr;
    DBHandler& m_db;
    std::atomic<bool> m_running;
    std::mutex m_lock;

};
