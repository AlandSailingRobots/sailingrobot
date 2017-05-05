/****************************************************************************************
*
* File:
* 		StateEstimationNodeSuite.h
*
* Purpose:
*		A set of unit tests for checking if the StateEstimationNode works as intended
*
* Developer Notes:
*
*
***************************************************************************************/

#pragma once

#include "Nodes/StateEstimationNode.h"
#include "../cxxtest/cxxtest/TestSuite.h"
#include "Messages/StateMessage.h"
#include "../../MessageBus/MessageBus.h"
#include "TestMocks/MessageLogger.h"
#include "TestMocks/StateMessageListener.h"
#include "Messages/CompassDataMsg.h"
#include "Messages/GPSDataMsg.h"


// For std::this_thread
#include <chrono>
#include <thread>

#define STATE_ESTIMATIONODE_TEST_COUNT   8


class StateEstimationNodeSuite : public CxxTest::TestSuite
{
public:

  StateEstimationNode* sEstimationNode;
  StateMessageListener* stateMessageListener;
  int speedLimit = 1;

  std::thread* thr;
  MessageLogger* logger;
  int testCount = 0;
  // Cheeky method for declaring and initialising a static in a header file
  static MessageBus& msgBus(){
    static MessageBus* mbus = new MessageBus();
    return *mbus;
  }

  static void runMessageLoop()
  {
    msgBus().run();
  }

  void setUp()
  {
    // setup them up once in this test, delete them when the program closes
    if(sEstimationNode == 0)
    {
      Logger::DisableLogging();
      logger = new MessageLogger(msgBus());
      stateMessageListener = new StateMessageListener(msgBus());
      srand (time(NULL));
      thr = new std::thread(runMessageLoop);
    }
    sEstimationNode = new StateEstimationNode(msgBus(), .5, speedLimit);
    stateMessageListener->resetStateDataReceived();
    testCount++;
  }

  void tearDown()
  {
    delete sEstimationNode;
    if(testCount == STATE_ESTIMATIONODE_TEST_COUNT)
    {
      delete logger;
      delete stateMessageListener;
    }
  }

  void test_StateEstimationNodeInit(){
    TS_ASSERT(sEstimationNode->init());
  }

  void test_StateEstimationNodeStateMsgReceived(){
    sEstimationNode->start();
    std::this_thread::sleep_for(std::chrono::milliseconds(2600));
    TS_ASSERT(logger->stateDataReceived());
    logger->clearState();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    TS_ASSERT(logger->stateDataReceived());
  }

  void test_StateMessageListener(){
    sEstimationNode->start();
    std::this_thread::sleep_for(std::chrono::milliseconds(3100));
    TS_ASSERT(logger->stateDataReceived());
    TS_ASSERT(stateMessageListener->stateDataReceived());
  }

  void test_StateEstimationHeading(){
    sEstimationNode->start();
    std::this_thread::sleep_for(std::chrono::milliseconds(2600));
    int nextDeclination = 10;
    MessagePtr wayPointMsgData = std::make_unique<WaypointDataMsg>(2, 19.81, 60.2, nextDeclination, 6, 15,  1, 19.82, 60.1, 6, 15);
    MessagePtr msgPtrWayPoint = std::move(std::move(wayPointMsgData));
    Message* msgWayPoint = msgPtrWayPoint.get();
    sEstimationNode->processMessage(msgWayPoint);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    TS_ASSERT(stateMessageListener->getVesselheading() == 0);
    int heading = 100;
    MessagePtr compassMsgData = std::make_unique<CompassDataMsg>(heading, 80, 60);
    MessagePtr msgPtr = std::move(std::move(compassMsgData));
    Message* msg = msgPtr.get();
    sEstimationNode->processMessage(msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    float vesselHeading = Utility::addDeclinationToHeading(heading, nextDeclination);
    float stateEstimationNodeVesselHeading = stateMessageListener->getVesselheading();
    TS_ASSERT(stateEstimationNodeVesselHeading == vesselHeading);
  }

  void test_StateEstimationStateMessageGPSData(){
    double latitude = 60.09726;
    double longitude = 19.93481;
    double unixTime = 1;
    double speed = 1;
    double heading = 10;
    int satCount = 2;
    GPSMode mode = GPSMode::LatLonOk;

    sEstimationNode->start();
    std::this_thread::sleep_for(std::chrono::milliseconds(2600));
    MessagePtr gpsMsgData = std::make_unique<GPSDataMsg>(false,false,latitude,longitude,unixTime,speed,heading,satCount,
      mode);
      MessagePtr msgPtr = std::move(std::move(gpsMsgData));
      Message* msg = msgPtr.get();
      sEstimationNode->processMessage(msg);
      std::this_thread::sleep_for(std::chrono::milliseconds(500));

      TS_ASSERT(stateMessageListener->stateDataReceived());
      TS_ASSERT(stateMessageListener->getVesselLat() == latitude);
      TS_ASSERT(stateMessageListener->getVesselLon() == longitude);
      TS_ASSERT(stateMessageListener->getVesselSpeed() == speed);
      TS_ASSERT(stateMessageListener->getVesselCourse() == heading);
    }

    void test_SEstStateMsgSpdAndDeclZero(){
      sEstimationNode->start();
      std::this_thread::sleep_for(std::chrono::milliseconds(2600));
      int nextDeclination = 0;
      MessagePtr wayPointMsgData = std::make_unique<WaypointDataMsg>(2, 19.81, 60.2, nextDeclination, 6, 15,  1, 19.82, 60.1, 6, 15);
      MessagePtr msgPtrWayPoint = std::move(std::move(wayPointMsgData));
      Message* msgWayPoint = msgPtrWayPoint.get();
      sEstimationNode->processMessage(msgWayPoint);
      std::this_thread::sleep_for(std::chrono::milliseconds(500));

      int headingComp = 0;
      MessagePtr compassMsgData = std::make_unique<CompassDataMsg>(headingComp, 80, 60);
      MessagePtr msgPtrComp = std::move(std::move(compassMsgData));
      Message* msgCompass = msgPtrComp.get();
      sEstimationNode->processMessage(msgCompass);
      std::this_thread::sleep_for(std::chrono::milliseconds(500));

      double latitude = 60.09726;
      double longitude = 19.93481;
      double unixTime = 1;
      double speed = 0;
      double headingGPS = 10;
      int satCount = 2;
      GPSMode mode = GPSMode::LatLonOk;
      MessagePtr gpsMsgData = std::make_unique<GPSDataMsg>(false,false,latitude,longitude,unixTime,speed,headingGPS,satCount,
        mode);
        MessagePtr msgPtrGpS = std::move(std::move(gpsMsgData));
        Message* msgGPS = msgPtrGpS.get();
        sEstimationNode->processMessage(msgGPS);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        TS_ASSERT(stateMessageListener->getVesselCourse() == 0);
      }

      void test_SEstStateMsgSpdAndDeclOverZero(){
        sEstimationNode->start();
        std::this_thread::sleep_for(std::chrono::milliseconds(2600));
        int nextDeclination = 10;
        MessagePtr wayPointMsgData = std::make_unique<WaypointDataMsg>(2, 19.81, 60.2, nextDeclination, 6, 15,  1, 19.82, 60.1, 6, 15);
        MessagePtr msgPtrWayPoint = std::move(std::move(wayPointMsgData));
        Message* msgWayPoint = msgPtrWayPoint.get();
        sEstimationNode->processMessage(msgWayPoint);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        TS_ASSERT(stateMessageListener->getVesselheading() == 0);
        int heading = 100;
        MessagePtr compassMsgData = std::make_unique<CompassDataMsg>(heading, 80, 60);
        MessagePtr msgPtr = std::move(std::move(compassMsgData));
        Message* msg = msgPtr.get();
        sEstimationNode->processMessage(msg);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        float stateEstimationNodeVesselHeading = stateMessageListener->getVesselheading();
        TS_ASSERT(stateEstimationNodeVesselHeading != 0);

        double latitude = 60.09726;
        double longitude = 19.93481;
        double unixTime = 1;
        double speed = 0.5;
        double headingGPS = 10;
        int satCount = 2;
        GPSMode mode = GPSMode::LatLonOk;
        MessagePtr gpsMsgData = std::make_unique<GPSDataMsg>(false,false,latitude,longitude,unixTime,speed,headingGPS,satCount,
          mode);
          MessagePtr msgPtrGpS = std::move(std::move(gpsMsgData));
          Message* msgGPS = msgPtrGpS.get();
          sEstimationNode->processMessage(msgGPS);
          std::this_thread::sleep_for(std::chrono::milliseconds(500));

          TS_ASSERT(stateMessageListener->getVesselCourse() > headingGPS);
        }

        void test_SEstStateMsgSpdLessThanZero(){
          sEstimationNode->start();
          std::this_thread::sleep_for(std::chrono::milliseconds(2600));
          int nextDeclination = 10;
          MessagePtr wayPointMsgData = std::make_unique<WaypointDataMsg>(2, 19.81, 60.2, nextDeclination, 6, 15,  1, 19.82, 60.1, 6, 15);
          MessagePtr msgPtrWayPoint = std::move(std::move(wayPointMsgData));
          Message* msgWayPoint = msgPtrWayPoint.get();
          sEstimationNode->processMessage(msgWayPoint);
          std::this_thread::sleep_for(std::chrono::milliseconds(500));

          TS_ASSERT(stateMessageListener->getVesselheading() == 0);
          int heading = 100;
          MessagePtr compassMsgData = std::make_unique<CompassDataMsg>(heading, 80, 60);
          MessagePtr msgPtr = std::move(std::move(compassMsgData));
          Message* msg = msgPtr.get();
          sEstimationNode->processMessage(msg);
          std::this_thread::sleep_for(std::chrono::milliseconds(500));
          float stateEstimationNodeVesselHeading = stateMessageListener->getVesselheading();
          TS_ASSERT(stateEstimationNodeVesselHeading != 0);

          double latitude = 60.09726;
          double longitude = 19.93481;
          double unixTime = 1;
          double speed = -1;
          double headingGPS = 10;
          int satCount = 2;
          GPSMode mode = GPSMode::LatLonOk;
          MessagePtr gpsMsgData = std::make_unique<GPSDataMsg>(false,false,latitude,longitude,unixTime,speed,headingGPS,satCount,
            mode);
            MessagePtr msgPtrGpS = std::move(std::move(gpsMsgData));
            Message* msgGPS = msgPtrGpS.get();
            sEstimationNode->processMessage(msgGPS);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));

            TS_ASSERT(stateMessageListener->getVesselCourse() == headingGPS);
          }

        };
