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

#include "../../MessageBus/MessageBus.h"
#include "../../Database/DBHandler.h"
#include "../../Math/Utility.h"
#include "MessageBusTestHelper.h"
#include "../../Messages/CompassDataMsg.h"
#include "../../Messages/GPSDataMsg.h"
#include "../../Messages/StateMessage.h"
#include "../../SystemServices/Timer.h"
#include "../../WorldState/StateEstimationNode.h"
#include "../cxxtest/cxxtest/TestSuite.h"
#include "TestMocks/MessageLogger.h"

#include <chrono>

#define STATE_ESTIMATIONODE_TEST_COUNT 9

class StateEstimationNodeSuite : public CxxTest::TestSuite {
   public:
    std::unique_ptr<StateEstimationNode> sEstimationNode;

    std::unique_ptr<DBHandler> dbhandler;
    std::unique_ptr<MockNode> mockNode;
    bool nodeRegistered = false;

    MessageBus messageBus;
    std::unique_ptr<MessageBusTestHelper> messageBusHelper;

    int testCount = 0;

    // ----------------
    // Setup the objects to test
    // ----------------
    void setUp() {
        // setup them up once in this test, delete them when the program closes
        if (sEstimationNode == 0) {
            mockNode.reset(new MockNode(messageBus, nodeRegistered));
            Logger::DisableLogging();
            dbhandler.reset(new DBHandler("../asr.db"));
            sEstimationNode.reset(new StateEstimationNode(messageBus, *dbhandler));
            sEstimationNode->start();
            std::this_thread::sleep_for(std::chrono::milliseconds(2600));
            messageBusHelper.reset(new MessageBusTestHelper(messageBus));
        }
        testCount++;
    }

    // ----------------
    // End of test when all test have been successfull
    // ----------------
    void tearDown() {
        if (testCount == STATE_ESTIMATIONODE_TEST_COUNT) {
            sEstimationNode->stop();
            messageBusHelper.reset();
        }
    }

    // ----------------
    // Test Initialisation of the object
    // ----------------
    void test_StateEstimationNodeInit() { TS_ASSERT(nodeRegistered); }

    // ----------------
    // Test for the absence of a returned message by a offline GPS
    // ----------------
    void test_StateEstimationNodeGPSNotOnline() {
        TS_ASSERT(sEstimationNode->init());
        TS_ASSERT(!mockNode->m_MessageReceived);
    }

    // ----------------
    // Test to see if a message concerning the node will be listened
    // ----------------
    void test_StateMessageListener() {
        double latitude = 60.09726;
        double longitude = 19.93481;
        double unixTime = 1;
        double speed = 1;
        double heading = 10;
        int satCount = 2;
        GPSMode mode = GPSMode::LatLonOk;

        MessagePtr gpsData = std::make_unique<GPSDataMsg>(false, true, latitude, longitude,
                                                          unixTime, speed, heading, satCount, mode);
        messageBus.sendMessage(std::move(gpsData));
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        TS_ASSERT(mockNode->m_MessageReceived);
    }

    // ----------------
    // Test to see if ,after the GPS messsage, the heading is not changed
    // ----------------
    void test_StateEstimationStateMsgHeading() {
        double latitude = 60.09726;
        double longitude = 19.93481;
        double unixTime = 1;
        double speed = 1;
        double headingGPS = 10;
        int satCount = 2;
        GPSMode mode = GPSMode::LatLonOk;

        MessagePtr gpsData = std::make_unique<GPSDataMsg>(
            false, true, latitude, longitude, unixTime, speed, headingGPS, satCount, mode);
        messageBus.sendMessage(std::move(gpsData));
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        int nextDeclination = 6;
        MessagePtr wayPointDataMsg = std::make_unique<WaypointDataMsg>(
            2, 19.922311, 60.105700, nextDeclination, 12, 3, false, 1, longitude, latitude, 6, 20);

        messageBus.sendMessage(std::move(wayPointDataMsg));
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        int heading = 100;
        MessagePtr compassData = std::make_unique<CompassDataMsg>(heading, 80, 60);
        messageBus.sendMessage(std::move(compassData));
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // Check if there is the same result by the processing next to the Compass data has been
        // received
        float vesselHeading = Utility::addDeclinationToHeading(heading, nextDeclination);
        float stateEstimationNodeVesselHeading = mockNode->m_StateMsgHeading;
        TS_ASSERT_EQUALS(stateEstimationNodeVesselHeading, vesselHeading);
    }

    // ----------------
    // Test to see if the message, sent by the node, is with the values
    // ----------------
    void test_StateEstimationStateMessageGPSData() {
        double latitude = 60.09726;
        double longitude = 19.93481;
        double unixTime = 1;
        double speed = -1;
        double heading = 190;
        int satCount = 2;
        GPSMode mode = GPSMode::LatLonOk;

        MessagePtr gpsData = std::make_unique<GPSDataMsg>(false, true, latitude, longitude,
                                                          unixTime, speed, heading, satCount, mode);
        messageBus.sendMessage(std::move(gpsData));
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        TS_ASSERT_EQUALS(mockNode->m_StateMsgLat, latitude);
        TS_ASSERT_EQUALS(mockNode->m_StateMsgLon, longitude);
        TS_ASSERT_EQUALS(mockNode->m_StateMsgSpeed, speed);
        TS_ASSERT_EQUALS(mockNode->m_StateMsgCourse, heading);
    }

    // ----------------
    // Test to see if the message, sent by the node, is with the values
    // ----------------
    void test_StateEstStateMsgSpeedAndDeclZero() {
        int nextDeclination = 0;
        MessagePtr wayPointMsgData = std::make_unique<WaypointDataMsg>(
            2, 19.81, 60.2, nextDeclination, 6, 15, true, 1, 19.82, 60.1, 6, 15);
        messageBus.sendMessage(std::move(wayPointMsgData));
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        int headingComp = 0;
        MessagePtr compassMsgData = std::make_unique<CompassDataMsg>(headingComp, 80, 60);
        messageBus.sendMessage(std::move(compassMsgData));
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        double latitude = 60.09726;
        double longitude = 19.93481;
        double unixTime = 1;
        double speed = 0;
        double headingGPS = 10;
        int satCount = 2;
        GPSMode mode = GPSMode::LatLonOk;
        MessagePtr gpsMsgData = std::make_unique<GPSDataMsg>(
            false, true, latitude, longitude, unixTime, speed, headingGPS, satCount, mode);
        messageBus.sendMessage(std::move(gpsMsgData));
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        TS_ASSERT_DELTA(mockNode->m_StateMsgCourse, 0, 1e-7);
    }

    void test_StateEstStateMsgSpeedAndDeclOverZero() {
        double latitude = 60.09726;
        double longitude = 19.93481;
        double unixTime = 1;
        double speed = 0.5;
        double headingGPS = 10;
        int satCount = 2;
        GPSMode mode = GPSMode::LatLonOk;

        MessagePtr gpsMsgData = std::make_unique<GPSDataMsg>(
            false, true, latitude, longitude, unixTime, speed, headingGPS, satCount, mode);
        messageBus.sendMessage(std::move(gpsMsgData));
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        int nextDeclination = 10;
        MessagePtr wayPointMsgData = std::make_unique<WaypointDataMsg>(
            2, 19.81, 60.2, nextDeclination, 6, 15, true, 1, 19.82, 60.1, 6, 15);
        messageBus.sendMessage(std::move(wayPointMsgData));
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        int heading = 100;
        MessagePtr compassMsgData = std::make_unique<CompassDataMsg>(heading, 80, 60);
        messageBus.sendMessage(std::move(compassMsgData));
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        float stateEstimationNodeVesselHeading = mockNode->m_StateMsgHeading;
        TS_ASSERT(stateEstimationNodeVesselHeading != 0);
        TS_ASSERT(mockNode->m_StateMsgCourse > headingGPS);
    }

    void test_StateEstStateMsgSpeedLessThanZero() {
        double latitude = 60.09726;
        double longitude = 19.93481;
        double unixTime = 1;
        double speed = -1;
        double headingGPS = 160.00;
        int satCount = 2;
        GPSMode mode = GPSMode::LatLonOk;

        MessagePtr gpsMsgData = std::make_unique<GPSDataMsg>(
            false, true, latitude, longitude, unixTime, speed, headingGPS, satCount, mode);
        messageBus.sendMessage(std::move(gpsMsgData));

        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        int nextDeclination = 10;
        MessagePtr wayPointMsgData = std::make_unique<WaypointDataMsg>(
            2, 19.81, 60.2, nextDeclination, 6, 15, true, 1, 19.82, 60.1, 6, 15);
        messageBus.sendMessage(std::move(wayPointMsgData));

        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        int heading = 100;
        MessagePtr compassMsgData = std::make_unique<CompassDataMsg>(heading, 80, 60);
        MessagePtr compassData = std::make_unique<CompassDataMsg>(heading, 80, 60);
        messageBus.sendMessage(std::move(compassData));

        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        float stateEstimationNodeVesselHeading = mockNode->m_StateMsgHeading;
        TS_ASSERT(stateEstimationNodeVesselHeading != 0);
        TS_ASSERT_DELTA(mockNode->m_StateMsgCourse, headingGPS, 1e-7);
    }

    // ----------------
    // Test for update frequency
    // ----------------
    void test_StateEstimationUpdateFrequency() {
        Timer timer;

        dbhandler->changeOneValue("config_vessel_state", "1", "0.7", "loop_time");
        MessagePtr serverConfig = std::make_unique<ServerConfigsReceivedMsg>();
        messageBus.sendMessage(std::move(serverConfig));

        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        TS_ASSERT(mockNode->m_MessageReceived);

        mockNode->m_MessageReceived = false;
        while (not mockNode->m_MessageReceived)
            ;

        timer.start();
        mockNode->m_MessageReceived = false;
        while (not mockNode->m_MessageReceived)
            ;
        timer.stop();

        TS_ASSERT_DELTA(timer.timePassed(), 0.70, 1e-2);

        dbhandler->changeOneValue("config_vessel_state", "1", "0.5", "loop_time");
    }
};
