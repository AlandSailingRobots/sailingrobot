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

#include "WorldState/StateEstimationNode.h"
#include "../cxxtest/cxxtest/TestSuite.h"
#include "Messages/StateMessage.h"
#include "../../MessageBus/MessageBus.h"
#include "TestMocks/MessageLogger.h"
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

  MockNode* mockNode;
  bool nodeRegistered = false;


  std::thread* thr;
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
    mockNode = new MockNode(msgBus(), nodeRegistered);
    // setup them up once in this test, delete them when the program closes
    if(sEstimationNode == 0)
    {
      Logger::DisableLogging();


      sEstimationNode = new StateEstimationNode(msgBus(), .5, 0, 1);
      sEstimationNode->start();
      std::this_thread::sleep_for(std::chrono::milliseconds(2600));

      thr = new std::thread(runMessageLoop);
    }
    testCount++;
  }

  void tearDown()
  {
    if(testCount == STATE_ESTIMATIONODE_TEST_COUNT)
    {
      delete sEstimationNode;
    }
    delete mockNode;
  }

  void test_StateEstimationNodeInit(){
    TS_ASSERT(nodeRegistered);
  }

  void test_StateEstimationNodeGPSNotOnline(){
    TS_ASSERT(sEstimationNode->init());
    TS_ASSERT(!mockNode->m_MessageReceived);

  }

  void test_StateMessageListener(){
    double latitude = 60.09726;
    double longitude = 19.93481;
    double unixTime = 1;
    double speed = 1;
    double heading = 10;
    int satCount = 2;
    GPSMode mode = GPSMode::LatLonOk;

    MessagePtr gpsData =  std::make_unique<GPSDataMsg>(false,true,latitude,longitude,unixTime,speed,heading,satCount,
      mode);
      msgBus().sendMessage(std::move(gpsData));
      std::this_thread::sleep_for(std::chrono::milliseconds(500));

      TS_ASSERT(mockNode->m_MessageReceived);
    }

    void test_StateEstimationStateMsgHeading(){
      double latitude = 60.09726;
      double longitude = 19.93481;
      double unixTime = 1;
      double speed = 1;
      double headingGPS = 10;
      int satCount = 2;
      GPSMode mode = GPSMode::LatLonOk;

      MessagePtr gpsData =  std::make_unique<GPSDataMsg>(false,true,latitude,longitude,unixTime,speed,headingGPS,satCount,
        mode);
        msgBus().sendMessage(std::move(gpsData));
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        int nextDeclination = 10;
        MessagePtr wayPointMsgData = std::make_unique<WaypointDataMsg>(2, 19.81, 60.2, nextDeclination, 6, 15,  1, 19.82, 60.1, 6, 15);
        msgBus().sendMessage(std::move(wayPointMsgData));
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        TS_ASSERT_DELTA(mockNode->m_StateMsgHeading, 0, 1e-7);

        int heading = 100;
        MessagePtr compassData =  std::make_unique<CompassDataMsg>(heading, 80, 60);
        msgBus().sendMessage(std::move(compassData));
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        float vesselHeading = Utility::addDeclinationToHeading(heading, nextDeclination);
        float stateEstimationNodeVesselHeading = mockNode->m_StateMsgHeading;
        TS_ASSERT_EQUALS(stateEstimationNodeVesselHeading, vesselHeading);
      }

      void test_StateEstimationStateMessageGPSData(){
        double latitude = 60.09726;
        double longitude = 19.93481;
        double unixTime = 1;
        double speed = 1;
        double heading = 10;
        int satCount = 2;
        GPSMode mode = GPSMode::LatLonOk;

        MessagePtr gpsData =  std::make_unique<GPSDataMsg>(false,true,latitude,longitude,unixTime,speed,heading,satCount,
          mode);
          msgBus().sendMessage(std::move(gpsData));
          std::this_thread::sleep_for(std::chrono::milliseconds(500));

          TS_ASSERT_EQUALS(mockNode->m_StateMsglLat, latitude);
          TS_ASSERT_EQUALS(mockNode->m_StateMsgLon, longitude);
          TS_ASSERT_EQUALS(mockNode->m_StateMsgSpeed, speed);
          TS_ASSERT_EQUALS(mockNode->m_StateMsgCourse, heading);
        }

        void test_StateEstStateMsgSpeedAndDeclZero(){
          int nextDeclination = 0;
          MessagePtr wayPointMsgData = std::make_unique<WaypointDataMsg>(2, 19.81, 60.2, nextDeclination, 6, 15,  1, 19.82, 60.1, 6, 15);
          msgBus().sendMessage(std::move(wayPointMsgData));
          std::this_thread::sleep_for(std::chrono::milliseconds(500));

          int headingComp = 0;
          MessagePtr compassMsgData = std::make_unique<CompassDataMsg>(headingComp, 80, 60);
          msgBus().sendMessage(std::move(compassMsgData));
          std::this_thread::sleep_for(std::chrono::milliseconds(500));

          double latitude = 60.09726;
          double longitude = 19.93481;
          double unixTime = 1;
          double speed = 0;
          double headingGPS = 10;
          int satCount = 2;
          GPSMode mode = GPSMode::LatLonOk;
          MessagePtr gpsMsgData = std::make_unique<GPSDataMsg>(false,true,latitude,longitude,unixTime,speed,headingGPS,satCount,
            mode);
            msgBus().sendMessage(std::move(gpsMsgData));
            std::this_thread::sleep_for(std::chrono::milliseconds(500));

            TS_ASSERT_DELTA(mockNode->m_StateMsgCourse, 0, 1e-7);
          }

          void test_StateEstStateMsgSpeedAndDeclOverZero(){
            double latitude = 60.09726;
            double longitude = 19.93481;
            double unixTime = 1;
            double speed = 0.5;
            double headingGPS = 10;
            int satCount = 2;
            GPSMode mode = GPSMode::LatLonOk;

            MessagePtr gpsMsgData = std::make_unique<GPSDataMsg>(false,true,latitude,longitude,unixTime,speed,headingGPS,satCount,
              mode);
              msgBus().sendMessage(std::move(gpsMsgData));
              std::this_thread::sleep_for(std::chrono::milliseconds(500));

              int nextDeclination = 10;
              MessagePtr wayPointMsgData = std::make_unique<WaypointDataMsg>(2, 19.81, 60.2, nextDeclination, 6, 15,  1, 19.82, 60.1, 6, 15);
              msgBus().sendMessage(std::move(wayPointMsgData));
              std::this_thread::sleep_for(std::chrono::milliseconds(500));

              int heading = 100;
              MessagePtr compassMsgData = std::make_unique<CompassDataMsg>(heading, 80, 60);
              msgBus().sendMessage(std::move(compassMsgData));
              std::this_thread::sleep_for(std::chrono::milliseconds(500));

              float stateEstimationNodeVesselHeading = mockNode->m_StateMsgHeading;
              TS_ASSERT(stateEstimationNodeVesselHeading != 0);
              TS_ASSERT(mockNode->m_StateMsgCourse > headingGPS);
            }

            void test_StateEstStateMsgSpeedLessThanZero(){
              double latitude = 60.09726;
              double longitude = 19.93481;
              double unixTime = 1;
              double speed = -1;
              double headingGPS = 10.00;
              int satCount = 2;
              GPSMode mode = GPSMode::LatLonOk;

              MessagePtr gpsMsgData = std::make_unique<GPSDataMsg>(false,true,latitude,longitude,unixTime,speed,headingGPS,satCount,
                mode);
                msgBus().sendMessage(std::move(gpsMsgData));

                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                int nextDeclination = 10;
                MessagePtr wayPointMsgData = std::make_unique<WaypointDataMsg>(2, 19.81, 60.2, nextDeclination, 6, 15,  1, 19.82, 60.1, 6, 15);
                msgBus().sendMessage(std::move(wayPointMsgData));

                std::this_thread::sleep_for(std::chrono::milliseconds(500));

                int heading = 100;
                MessagePtr compassMsgData = std::make_unique<CompassDataMsg>(heading, 80, 60);
                MessagePtr compassData =  std::make_unique<CompassDataMsg>(heading, 80, 60);
                msgBus().sendMessage(std::move(compassData));

                std::this_thread::sleep_for(std::chrono::milliseconds(500));

                float stateEstimationNodeVesselHeading = mockNode->m_StateMsgHeading;
                TS_ASSERT(stateEstimationNodeVesselHeading != 0);
                TS_ASSERT_DELTA(mockNode->m_StateMsgCourse, headingGPS, 1e-7);
              }

            };
