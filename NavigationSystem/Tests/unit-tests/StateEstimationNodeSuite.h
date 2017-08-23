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
#include "SystemServices/Timer.h"
#include "DataBase/DBHandler.h"
#include "Math/Utility.h"


// For std::this_thread
#include <chrono>
#include <thread>

#define STATE_ESTIMATIONODE_TEST_COUNT   8


class StateEstimationNodeSuite : public CxxTest::TestSuite {

public:

    StateEstimationNode* sEstimationNode;

    DBHandler* dbhandler;
    MockNode* mockNode;
    bool nodeRegistered = false;

    std::thread* thr;
    int testCount = 0;

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
    mockNode = new MockNode(msgBus(), nodeRegistered);
    // setup them up once in this test, delete them when the program closes
    if(sEstimationNode == 0){
        Logger::DisableLogging();
        dbhandler = new DBHandler("../asr.db");
      sEstimationNode = new StateEstimationNode(msgBus(),*dbhandler, .5, 0, 1);
      sEstimationNode->start();
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
    if(testCount == STATE_ESTIMATIONODE_TEST_COUNT)
    {
        sEstimationNode -> stop();
        msgBus().stop();
        thr->join();
        delete thr;
        delete sEstimationNode;
        delete dbhandler;
    }
    delete mockNode;
  }

    // ----------------
    // Test Initialisation of the object
    // ----------------
    void test_StateEstimationNodeInit()
    {
        TS_ASSERT(nodeRegistered);
    }

    // ----------------
    // Test for the absence of a returned message by a offline GPS
    // ----------------
    void test_StateEstimationNodeGPSNotOnline()
    {
        TS_ASSERT(sEstimationNode->init());
        TS_ASSERT(!mockNode->m_MessageReceived);
    }

    // ----------------
    // Test to see if a message concerning the node will be listened
    // ----------------
    void test_StateMessageListener()
    {
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

  // ----------------
  // Test to see if ,after the GPS messsage, the heading is not changed
  // ----------------
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
        // TODO : Check the constructor because the variables seems not appropriate.
        MessagePtr wayPointMsgData = std::make_unique<WaypointDataMsg>(2, 19.81, 60.2, nextDeclination, 6, 15,  1, 19.82, 60.1, 6, 15);
        msgBus().sendMessage(std::move(wayPointMsgData));
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        TS_ASSERT_DELTA(mockNode->m_StateMsgHeading, 0, 1e-7); // Check if Heading of the state message is null because we must wait a data from the compass message

        int heading = 100;
        MessagePtr compassData =  std::make_unique<CompassDataMsg>(heading, 80, 60);
        msgBus().sendMessage(std::move(compassData));
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        //Check if there is the same result by the processing next to the Compass data has been received
        float vesselHeading = Utility::addDeclinationToHeading(heading, nextDeclination);
        float stateEstimationNodeVesselHeading = mockNode->m_StateMsgHeading;
        TS_ASSERT_EQUALS(stateEstimationNodeVesselHeading, vesselHeading);
    }

      // ----------------
      // Test to see if the message, sent by the node, is with the values
      // ----------------
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

        TS_ASSERT_EQUALS(mockNode->m_StateMsgLat, latitude);
        TS_ASSERT_EQUALS(mockNode->m_StateMsgLon, longitude);
        TS_ASSERT_EQUALS(mockNode->m_StateMsgSpeed, speed);
        TS_ASSERT_EQUALS(mockNode->m_StateMsgCourse, heading);
    }

    // ----------------
    // Test to see if the message, sent by the node, is with the values
    // ----------------
    void test_StateEstStateMsgSpeedAndDeclZero()
    {
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

    void test_StateEstStateMsgSpeedAndDeclOverZero()
    {
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

    void test_StateEstStateMsgSpeedLessThanZero()
    {
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

    // ----------------
    // Test for update frequency
    // ----------------
    void test_StateEstimationUpdateFrequency(){
        Timer timer;

        dbhandler->changeOneValue("config_vessel_state","1","0.7","loop_time");
        MessagePtr serverConfig = std::make_unique<ServerConfigsReceivedMsg>();
        msgBus().sendMessage(std::move(serverConfig));
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        TS_ASSERT(mockNode->m_MessageReceived);

        mockNode->m_MessageReceived = false;
        while(not mockNode->m_MessageReceived);
        timer.start();
        mockNode->m_MessageReceived = false;
        while(not mockNode->m_MessageReceived);
        timer.stop();

        TS_ASSERT_DELTA(timer.timePassed(), 0.70, 1e-2);

        // double heading = 10;
        // double speed = 1;
        // MaxRudAng = 20.0;
        // double desiredcourse = 15;
        //
        // MessagePtr stateData = std::make_unique<StateMessage>(heading,60.09726,19.93481,speed,0);
        // msgBus().sendMessage(std::move(stateData));
        // std::this_thread::sleep_for(std::chrono::milliseconds(700));
        //
        // MessagePtr desiredCourseData = std::make_unique<DesiredCourseMsg>(desiredcourse);
        // msgBus().sendMessage(std::move(desiredCourseData));
        // std::this_thread::sleep_for(std::chrono::milliseconds(700));
        //
        // double diffHeading = Utility::limitAngleRange(heading)-Utility::limitAngleRange(desiredcourse);
        // int rudderAngle = Utility::sgn(speed)*sin(Utility::degreeToRadian(diffHeading))*MaxRudAng;
        // std::this_thread::sleep_for(std::chrono::milliseconds(700));
        // double courseRegulatorRudderAngle = mockNode->m_rudderPosition;
        // TS_ASSERT_DELTA(courseRegulatorRudderAngle, rudderAngle, 1e-7);
        dbhandler->changeOneValue("config_vessel_state","1","0.5","loop_time");
    }

};
