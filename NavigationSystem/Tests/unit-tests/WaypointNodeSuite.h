/****************************************************************************************
 *
 * File:
 * 		WaypointNodeSuite.h
 *
 * Purpose:
 *		A set of unit tests for checking whether the WaypointNode is sending data
 *      correctly
 *
 * Developer Notes:
 *  - ./update_waypoints.py Mission/Unit_tests.json needs to be run before testing.
 *
 *
 *
 *	Functions that have tests:		Functions that does not have tests:
 *
 *	init 							processGPSMessage
 *									waypointReached
 *									sendMessage
 *									harvestWaypoint
 *
 ***************************************************************************************/

#pragma once

#include "../../MessageBus/MessageBus.h"
#include "MessageBusTestHelper.h"
#include "../../Messages/GPSDataMsg.h"
#include "../../Navigation/WaypointMgrNode.h"
#include "../cxxtest/cxxtest/TestSuite.h"
#include "TestMocks/MockNode.h"

#include <chrono>

#define WAYPOINT_TEST_COUNT 4

class WaypointNodeSuite : public CxxTest::TestSuite {
   public:
    WaypointMgrNode* waypoint;
    MockNode* mockNode;
    DBHandler* dbHandler;

    bool nodeRegistered;
    int testCount = 0;
    const int WAIT_FOR_MESSAGE = 500;

    MessageBus messageBus;
    std::unique_ptr<MessageBusTestHelper> messageBusHelper;

    void setUp() {
        // Only want to setup them up once in this test, only going to delete them when the program
        // closes and the OS destroys the process's memory
        if (waypoint == 0) {
            mockNode = new MockNode(messageBus, nodeRegistered);
            dbHandler = new DBHandler("../asr.db");
            // Reset of the database after reaching waypoints during test
            dbHandler->updateTable("current_mission", "harvested", "0", "1");
            dbHandler->updateTable("current_mission", "harvested", "0", "2");
            dbHandler->updateTable("current_mission", "harvested", "0", "3");
            Logger::DisableLogging();
            waypoint = new WaypointMgrNode(messageBus, *dbHandler);
            messageBusHelper.reset(new MessageBusTestHelper(messageBus));
        }
        testCount++;
    }

    void tearDown() {
        if (testCount == WAYPOINT_TEST_COUNT) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            messageBusHelper.reset();
            delete waypoint;
            delete dbHandler;
            delete mockNode;
        }
    }

    void test_WaypointNodeInitListener() {
        TS_ASSERT(waypoint->init());
        TS_ASSERT(nodeRegistered);

        std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_FOR_MESSAGE));
        TS_ASSERT(mockNode->m_MessageReceived);
        TS_ASSERT_EQUALS(mockNode->m_waypointNextId, 1);
        TS_ASSERT_DELTA(mockNode->m_waypointNextLongitude, 19.921311, 1e-8);
        TS_ASSERT_DELTA(mockNode->m_waypointNextLatitude, 60.107240, 1e-8);
        TS_ASSERT_EQUALS(mockNode->m_waypointNextDeclination, 6);
        TS_ASSERT_EQUALS(mockNode->m_waypointNextRadius, 20);
        TS_ASSERT_EQUALS(mockNode->m_waypointStayTime, 0);
        TS_ASSERT_EQUALS(mockNode->m_waypointPrevId, 0);
        TS_ASSERT_DELTA(mockNode->m_waypointPrevLongitude, 0, 1e-3);
        TS_ASSERT_DELTA(mockNode->m_waypointPrevLatitude, 0, 1e-4);
        TS_ASSERT_EQUALS(mockNode->m_waypointPrevDeclination, 0);
        TS_ASSERT_EQUALS(mockNode->m_waypointPrevRadius, 0);
    }

    void test_WaypointNodeCloseWaypoint() {
        double lat = 60.107240;
        double lon = 19.921311;

        MessagePtr stateMsg = std::make_unique<StateMessage>(0, lat, lon, 1, 1);
        messageBus.sendMessage(std::move(stateMsg));

        std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_FOR_MESSAGE));

        bool proofWaypointReached =
            dbHandler->retrieveCellAsInt("current_mission", "1", "harvested");
        TS_ASSERT_EQUALS(proofWaypointReached, true);

        TS_ASSERT(mockNode->m_MessageReceived);
        TS_ASSERT_EQUALS(mockNode->m_waypointNextId, 2);
        TS_ASSERT_DELTA(mockNode->m_waypointNextLongitude, 19.922311, 1e-8);
        TS_ASSERT_DELTA(mockNode->m_waypointNextLatitude, 60.105700, 1e-8);
        TS_ASSERT_EQUALS(mockNode->m_waypointNextDeclination, 6);
        TS_ASSERT_EQUALS(mockNode->m_waypointNextRadius, 12);
        TS_ASSERT_EQUALS(mockNode->m_waypointStayTime, 4);
        TS_ASSERT_EQUALS(mockNode->m_waypointPrevId, 1);
        TS_ASSERT_DELTA(mockNode->m_waypointPrevLongitude, lon, 1e-3);
        TS_ASSERT_DELTA(mockNode->m_waypointPrevLatitude, lat, 1e-4);
        TS_ASSERT_EQUALS(mockNode->m_waypointPrevDeclination, 6);
        TS_ASSERT_EQUALS(mockNode->m_waypointPrevRadius, 20);
    }

    void test_WaypointNodeReachWaypointStaytime() {
        double lat = 60.105700;
        double lon = 19.922311;
        int stayTime = 3;

        MessagePtr stateMsg = std::make_unique<StateMessage>(0, lat, lon, 1, 1);
        messageBus.sendMessage(std::move(stateMsg));
        std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_FOR_MESSAGE));
        mockNode->clearMessageReceived();

        std::this_thread::sleep_for(std::chrono::seconds(stayTime / 2));
        TS_ASSERT(not mockNode->m_MessageReceived);
        std::this_thread::sleep_for(std::chrono::seconds(stayTime));

        MessagePtr nextStateMsg = std::make_unique<StateMessage>(0, lat, lon, 1, 1);
        messageBus.sendMessage(std::move(nextStateMsg));
        std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_FOR_MESSAGE));

        bool proofWaypointReached =
            dbHandler->retrieveCellAsInt("current_mission", "2", "harvested");
        TS_ASSERT_EQUALS(proofWaypointReached, true);

        TS_ASSERT(mockNode->m_MessageReceived);
        TS_ASSERT_EQUALS(mockNode->m_waypointNextId, 3);
        TS_ASSERT_DELTA(mockNode->m_waypointNextLongitude, 19.921925, 1e-8);
        TS_ASSERT_DELTA(mockNode->m_waypointNextLatitude, 60.103818, 1e-8);
        TS_ASSERT_EQUALS(mockNode->m_waypointNextDeclination, 6);
        TS_ASSERT_EQUALS(mockNode->m_waypointNextRadius, 10);
        TS_ASSERT_EQUALS(mockNode->m_waypointStayTime, 4);
        TS_ASSERT_EQUALS(mockNode->m_waypointPrevId, 2);
        TS_ASSERT_DELTA(mockNode->m_waypointPrevLongitude, lon, 1e-3);
        TS_ASSERT_DELTA(mockNode->m_waypointPrevLatitude, lat, 1e-4);
        TS_ASSERT_EQUALS(mockNode->m_waypointPrevDeclination, 6);
        TS_ASSERT_EQUALS(mockNode->m_waypointPrevRadius, 12);
    }
};
