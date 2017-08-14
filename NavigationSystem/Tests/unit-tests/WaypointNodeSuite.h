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
 *  - ./insertPredefinedWaypoints.sh  needs to be run first.
 *
 *							12.4.17 JM
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

 #include "../cxxtest/cxxtest/TestSuite.h"
 #include "MessageBus/MessageBus.h"
 #include "TestMocks/MockNode.h"
 #include "Navigation/WaypointMgrNode.h"
 #include "Messages/GPSDataMsg.h"

// For std::this_thread
#include <chrono>
#include <thread>

#define WAIT_FOR_MESSAGE		500
#define WAYPOINT_TEST_COUNT     5

 class WaypointNodeSuite : public CxxTest::TestSuite {
    public:
    WaypointMgrNode* waypoint;
    MockNode* mockNode;
    bool nodeRegistered;
    std::thread* thr;
    int testCount = 0;
	DBHandler* dbHandler;

        	// Cheeky method for declaring and initialising a static in a header file
	static MessageBus& msgBus()
	{
		static MessageBus* mbus = new MessageBus();
		return *mbus;
	}

	static void runMessageLoop()
	{
		msgBus().run();
	}

	void setUp()
	{
		// Only want to setup them up once in this test, only going to delete them when the program closes and the OS destroys
		// the process's memory
        mockNode = new MockNode(msgBus(),nodeRegistered);
		if(waypoint == 0)
		{
			dbHandler = new DBHandler("../asr.db");
            //Reset of the database after reaching waypoints during test
            dbHandler->updateTable("current_mission","harvested","0","1");
            dbHandler->updateTable("current_mission","harvested","0","2");
            dbHandler->updateTable("current_mission","harvested","0","3");
            Logger::DisableLogging();
			waypoint = new WaypointMgrNode(msgBus(), *dbHandler);
            thr = new std::thread(runMessageLoop);
		}
		testCount++;
	}

	void tearDown()
	{
        delete mockNode;
		if(testCount == WAYPOINT_TEST_COUNT)
		{
            std::this_thread::sleep_for(std::chrono::seconds(1));
            msgBus().stop();
            thr->join();
            delete thr;
			delete waypoint;
			delete dbHandler;
		}
	}

    void test_WaypointNodeInitListener()
    {
        TS_ASSERT(waypoint->init());
        TS_ASSERT(nodeRegistered);
        std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_FOR_MESSAGE));
        TS_ASSERT(mockNode->m_MessageReceived);
        TS_ASSERT_EQUALS(mockNode->m_waypointNextId,1);
        TS_ASSERT_DELTA(mockNode->m_waypointNextLongitude,19.53291333,1e-8);
        TS_ASSERT_DELTA(mockNode->m_waypointNextLatitude,60.22654833,1e-8);
        TS_ASSERT_EQUALS(mockNode->m_waypointNextDeclination,6);
        TS_ASSERT_EQUALS(mockNode->m_waypointNextRadius,15);
        TS_ASSERT_EQUALS(mockNode->m_waypointStayTime,0);
        TS_ASSERT_EQUALS(mockNode->m_waypointPrevId,0);
        TS_ASSERT_DELTA(mockNode->m_waypointPrevLongitude,0,1e-3);
        TS_ASSERT_DELTA(mockNode->m_waypointPrevLatitude,0,1e-4);
        TS_ASSERT_EQUALS(mockNode->m_waypointPrevDeclination,0);
        TS_ASSERT_EQUALS(mockNode->m_waypointPrevRadius,0);
    }

    void test_WaypointNodeGpsMsg()
    {
        // Work if the waypoints correspond to the value on the DB
        double gpsLat = 60.45845845;
        double gpsLon = 45.23523523;
        MessagePtr gpsData = std::make_unique<GPSDataMsg>(true,true,gpsLat,gpsLon,12.12,1.7,273.0,2,GPSMode::LatLonOk);
        msgBus().sendMessage(std::move(gpsData));
        std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_FOR_MESSAGE));

        TS_ASSERT_DELTA(mockNode->m_Lat,gpsLat,1e-8);
        TS_ASSERT_DELTA(mockNode->m_Lon,gpsLon,1e-8);

        std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_FOR_MESSAGE));
        TS_ASSERT(mockNode->m_MessageReceived);
        TS_ASSERT_EQUALS(mockNode->m_waypointNextId,1);
        TS_ASSERT_DELTA(mockNode->m_waypointNextLongitude,19.53291333,1e-8);
        TS_ASSERT_DELTA(mockNode->m_waypointNextLatitude,60.22654833,1e-8);
        TS_ASSERT_EQUALS(mockNode->m_waypointNextDeclination,6);
        TS_ASSERT_EQUALS(mockNode->m_waypointNextRadius,15);
        TS_ASSERT_EQUALS(mockNode->m_waypointStayTime,0);
        TS_ASSERT_EQUALS(mockNode->m_waypointPrevId,0);
        TS_ASSERT_DELTA(mockNode->m_waypointPrevLongitude,0,1e-3);
        TS_ASSERT_DELTA(mockNode->m_waypointPrevLatitude,0,1e-4);
        TS_ASSERT_EQUALS(mockNode->m_waypointPrevDeclination,0);
        TS_ASSERT_EQUALS(mockNode->m_waypointPrevRadius,0);
    }

    void test_WaypointNodeCloseWaypoint(){
        double gpsLat = 60.22650000;
        double gpsLon = 19.53290000;

        // Work if the waypoints correspond to the value on the DB
        MessagePtr gpsData = std::make_unique<GPSDataMsg>(true,true,gpsLat,gpsLon,12.12,1.7,273,2,GPSMode::LatLonOk);
        msgBus().sendMessage(std::move(gpsData));
        std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_FOR_MESSAGE));

        bool proofWaypointReached = dbHandler->retrieveCellAsInt("current_mission","harvested","1");
        TS_ASSERT_EQUALS(proofWaypointReached,true);

        TS_ASSERT(mockNode->m_MessageReceived);
        TS_ASSERT_EQUALS(mockNode->m_waypointNextId,2);
        TS_ASSERT_DELTA(mockNode->m_waypointNextLongitude,19.51169000,1e-8);
        TS_ASSERT_DELTA(mockNode->m_waypointNextLatitude,60.21526667,1e-8);
        TS_ASSERT_EQUALS(mockNode->m_waypointNextDeclination,6);
        TS_ASSERT_EQUALS(mockNode->m_waypointNextRadius,15);
        TS_ASSERT_EQUALS(mockNode->m_waypointStayTime,2);
        TS_ASSERT_EQUALS(mockNode->m_waypointPrevId,2); // not normal
        TS_ASSERT_DELTA(mockNode->m_waypointPrevLongitude,gpsLon,1e-3);
        TS_ASSERT_DELTA(mockNode->m_waypointPrevLatitude,gpsLat,1e-4);
        TS_ASSERT_EQUALS(mockNode->m_waypointPrevDeclination,6);
        TS_ASSERT_EQUALS(mockNode->m_waypointPrevRadius,15);
    }

    void test_WaypointNodeReachWaypointStaytime(){
        double gpsLat = 60.21526667;
        double gpsLon = 19.51169000;
        int stayTime = 2; //current staytime of the waypoint

        // Work if the waypoints correspond to the value on the DB
        MessagePtr gpsData = std::make_unique<GPSDataMsg>(true,true,gpsLat,gpsLon,12.12,1.7,273,2,GPSMode::LatLonOk);
        msgBus().sendMessage(std::move(gpsData));
        std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_FOR_MESSAGE));

        mockNode->m_MessageReceived = false;
        std::this_thread::sleep_for(std::chrono::seconds(stayTime/2));
        TS_ASSERT(not mockNode->m_MessageReceived);
        std::this_thread::sleep_for(std::chrono::seconds(stayTime/2));

        MessagePtr nextGpsData = std::make_unique<GPSDataMsg>(true,true,gpsLat,gpsLon,12.12,1.7,273,2,GPSMode::LatLonOk);
        msgBus().sendMessage(std::move(nextGpsData));
        std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_FOR_MESSAGE));

        bool proofWaypointReached = dbHandler->retrieveCellAsInt("current_mission","harvested","2");
        TS_ASSERT_EQUALS(proofWaypointReached,true);

        TS_ASSERT(mockNode->m_MessageReceived);
        TS_ASSERT_EQUALS(mockNode->m_waypointNextId,3);
        TS_ASSERT_DELTA(mockNode->m_waypointNextLongitude,19.487045,1e-8);
        TS_ASSERT_DELTA(mockNode->m_waypointNextLatitude,60.20231833,1e-8);
        TS_ASSERT_EQUALS(mockNode->m_waypointNextDeclination,6);
        TS_ASSERT_EQUALS(mockNode->m_waypointNextRadius,15);
        TS_ASSERT_EQUALS(mockNode->m_waypointStayTime,3);
        TS_ASSERT_EQUALS(mockNode->m_waypointPrevId,3); //not normal
        TS_ASSERT_DELTA(mockNode->m_waypointPrevLongitude,gpsLon,1e-3);
        TS_ASSERT_DELTA(mockNode->m_waypointPrevLatitude,gpsLat,1e-4);
        TS_ASSERT_EQUALS(mockNode->m_waypointPrevDeclination,6);
        TS_ASSERT_EQUALS(mockNode->m_waypointPrevRadius,15);
    }

    void test_WaypointNodeReachOutWaypointDuringStaytime(){
        double gpsLat = 60.20231833;
        double gpsLon = 19.487045;

        double movedGpsLat = 60.20;
        double movedGpsLon = 19.48;

        int stayTime = 3; //current staytime of the waypoint

        // Work if the waypoints correspond to the value on the DB
        MessagePtr gpsData = std::make_unique<GPSDataMsg>(true,true,gpsLat,gpsLon,12.12,1.7,273,2,GPSMode::LatLonOk);
        msgBus().sendMessage(std::move(gpsData));
        std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_FOR_MESSAGE));
        std::this_thread::sleep_for(std::chrono::seconds(stayTime/3));

        MessagePtr nextGpsData = std::make_unique<GPSDataMsg>(true,true,movedGpsLat,movedGpsLon,12.12,1.7,273,2,GPSMode::LatLonOk);
        msgBus().sendMessage(std::move(nextGpsData));
        std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_FOR_MESSAGE));
        std::this_thread::sleep_for(std::chrono::seconds(stayTime/3));

        MessagePtr lastGpsData = std::make_unique<GPSDataMsg>(true,true,gpsLat,gpsLon,12.12,1.7,273,2,GPSMode::LatLonOk);
        msgBus().sendMessage(std::move(lastGpsData));
        std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_FOR_MESSAGE));
        std::this_thread::sleep_for(std::chrono::seconds(stayTime/3));

        bool proofWaypointReached = dbHandler->retrieveCellAsInt("current_mission","harvested","2");
        TS_ASSERT_EQUALS(proofWaypointReached,true);

        TS_ASSERT(mockNode->m_MessageReceived);
        TS_ASSERT_EQUALS(mockNode->m_waypointNextId,4);
        TS_ASSERT_DELTA(mockNode->m_waypointNextLongitude,19.471833335,1e-8);
        TS_ASSERT_DELTA(mockNode->m_waypointNextLatitude,60.1851725,1e-8);
        TS_ASSERT_EQUALS(mockNode->m_waypointNextDeclination,6);
        TS_ASSERT_EQUALS(mockNode->m_waypointNextRadius,15);
        TS_ASSERT_EQUALS(mockNode->m_waypointStayTime,0);
        TS_ASSERT_EQUALS(mockNode->m_waypointPrevId,4); //not normal
        TS_ASSERT_DELTA(mockNode->m_waypointPrevLongitude,gpsLon,1e-3);
        TS_ASSERT_DELTA(mockNode->m_waypointPrevLatitude,gpsLat,1e-4);
        TS_ASSERT_EQUALS(mockNode->m_waypointPrevDeclination,6);
        TS_ASSERT_EQUALS(mockNode->m_waypointPrevRadius,15);
    }
 };
