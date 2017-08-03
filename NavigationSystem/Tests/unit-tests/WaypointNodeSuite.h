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
 #include "TestMocks/MessageLogger.h"
 #include "Navigation/WaypointMgrNode.h"

// For std::this_thread
#include <chrono>
#include <thread>

#define WAIT_FOR_MESSAGE		300
#define WAYPOINT_TEST_COUNT     1

 class WaypointNodeSuite : public CxxTest::TestSuite {
    public:
    WaypointMgrNode* waypoint;
    MessageLogger* logger;
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
		if(waypoint == 0)
		{
			dbHandler = new DBHandler("../asr.db");
            Logger::DisableLogging();
			logger = new MessageLogger(msgBus());
			waypoint = new WaypointMgrNode(msgBus(), *dbHandler);
            thr = new std::thread(runMessageLoop);
		}
		testCount++;
        std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_FOR_MESSAGE));
	}

	void tearDown()
	{
		if(testCount == WAYPOINT_TEST_COUNT)
		{
            msgBus().stop();
            thr->join();
            delete thr;
			delete waypoint;
			delete logger;
			delete dbHandler;
		}
	}

    void test_WaypointNodeInit()
    {
        TS_ASSERT(waypoint->init());

        std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_FOR_MESSAGE));

        TS_ASSERT(logger->waypointDataReceived());
    }
 };
