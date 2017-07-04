/****************************************************************************************
*
* File:
* 		SailControlNodeSuite.h
*
* Purpose:
*		A set of unit tests for checking if the SailControlNode works as intended
*
* Developer Notes:
*
*
***************************************************************************************/

#pragma once

#include "WorldState/SailControlNode.h"
#include "../cxxtest/cxxtest/TestSuite.h"
#include "Messages/ActuatorPositionMsg.h"
#include "../../MessageBus/MessageBus.h"
#include "TestMocks/MessageLogger.h"
#include "Messages/WindStateMsg.h"
#include "Messages/StateMessage.h"


// For std::this_thread
#include <chrono>
#include <thread>
#include "All.h"
#include <math>

#define SAIL_CONTROLNODE_TEST_COUNT   8


class SailControlNodeSuite : public CxxTest::TestSuite
{
public:

  SailControlNode* sControlNode;

  MockNode* mockNode;
  bool nodeRegistered = false;
  double loopTime = 0.5;

  std::thread* thr;
  int testCount = 0;
  // Cheeky method for declaring and initialising a static in a header file
  static MessageBus& msgBus(){
    static MessageBus* mbus = new MessageBus();
    return *mbus;
  }

  // ----------------
  // Send messages 
  // ----------------
  static void runMessageLoop()
  {
    msgBus().run();
  }

  // ----------------
  // Setup the objects to test
  // ----------------
  void setUp()
  {
    // Test Node for message
    mockNode = new MockNode(msgBus(), nodeRegistered);
    // setup them up once in this test, delete them when the program closes
    if(sControlNode == 0)
    {
        dbHandler = new DBHandler("./asr.db");
        Logger::DisableLogging();


        sControlNode = new SailControlNode(msgBus(), .5,.79,.01,.1, 0, 0, dbHandler);
        sControlNode->start();
        std::this_thread::sleep_for(std::chrono::milliseconds(2600));

        thr = new std::thread(runMessageLoop);
    }
    testCount++;
  }

  // ----------------
  // End of test when all test have been successfull
  // ----------------
  void tearDown()
  {
    if(testCount == SAIL_CONTROLNODE_TEST_COUNT)
    {
      delete sControlNode;
    }
    delete mockNode;

  }

  // ----------------
  // Test Initialisation of the object
  // ----------------
  void test_SailControlNodeInit(){
    TS_ASSERT(nodeRegistered);
  }

  // ----------------
  // Test for the absence of a returned message by a offline GPS
  // ----------------
  void test_SailControlNode(){
    TS_ASSERT(sControlNode->init());
    TS_ASSERT(!mockNode->m_MessageReceived);

  }

  // ----------------
  // Test to see if a message concerning the node will be listened
  // ----------------
  void test_SailControlMessageListener(){
    double trueWindSpeed = 60.09726; //units ?
    double trueWindDirection = 19.93481;
    double apparentWindSpeed = 1; //units ?
    double apparentWindDirection = 1;
    
    MessagePtr windData =  std::make_unique<WindStateMsg>(trueWindSpeed,trueWindDirection,apparentWindSpeed,apparentWindDirection);
      msgBus().sendMessage(std::move(windData));
      std::this_thread::sleep_for(std::chrono::milliseconds(500));

      TS_ASSERT(mockNode->m_MessageReceived);
  }

  // ----------------
  // Test to see if ,after the GPS messsage, the heading is not changed
  // ----------------
  void test_SailControlNodeSailAngle(){
    double trueWindSpeed = 60.09726; //units ? m/s
    double trueWindDirection = 19.93481;
    double apparentWindSpeed = 1; //units ? M/s
    double apparentWindDirection = 50;
    
      MessagePtr windData =  std::make_unique<WindStateMsg>(trueWindSpeed,trueWindDirection,apparentWindSpeed,apparentWindDirection);
      msgBus().sendMessage(std::move(windData));
      std::this_thread::sleep_for(std::chrono::milliseconds(500));

        //Check if there is the same result by the processing next to the Compass data has been received
        float sailAngle = -sgn(m_ApparentWind)*(((m_MinSailAngle-m_MaxSailAngle)*std::abs(m_ApparentWind)/M_PI)+m_MaxSailAngle);
        float sailControlNodeSailAngle = mockNode->m_sailPosition;
        TS_ASSERT_EQUALS(sailControlNodeSailAngle, sailAngle);
      }

        // ----------------
        // Test to see if the message, sent by the node, is with the values
        // ----------------
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
            //Check if the Course keep a null value
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

              float sailControlNodeVesselHeading = mockNode->m_StateMsgHeading;
              // Check the modification and the new course
              TS_ASSERT(sailControlNodeVesselHeading != 0);
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

                float sailControlNodeVesselHeading = mockNode->m_StateMsgHeading;
                //CHeck the modification of the heading and the same result of heading because of the negative speed
                TS_ASSERT(sailControlNodeVesselHeading != 0);
                TS_ASSERT_DELTA(mockNode->m_StateMsgCourse, headingGPS, 1e-7);
              }

            };
