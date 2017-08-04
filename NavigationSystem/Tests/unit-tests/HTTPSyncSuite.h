/****************************************************************************************
 *
 * File:
 * 		HTTPSyncSuite.h
 *
 * Purpose:
 *
 *
 * Developer Notes:
 *  - Still copypasting
 *  - Database "asr.db" needs to be created and initialized before testing. Run:
 *      - ./installdb.sh  -  Server address needs to be "http://www.sailingrobots.com/testdata/sync/" or "http://localhost/Remote-sailing-robots/sync/"
 *      - ./insertPredefinedWaypoints.sh  -  Choose a waypoint that works
 *
 *
 *                          11.4.17 JM
 *
 *  Functions that have tests:      Functions that does not have tests:
 *
 *  init                            start
 *  pushDatalogs                    private functions
 *  pushWaypoints
 *  pushConfigs
 *  getWaypointsFromServer
 *  getConfigsFromServer
 *
 ***************************************************************************************/

#pragma once

#include "../cxxtest/cxxtest/TestSuite.h"
#include "HTTPSync/HTTPSyncNode.h"
#include "MessageBus/MessageBus.h"
#include "SystemServices/Logger.h"
#include "DataBase/DBHandler.h"
#include "DataBase/DBLoggerNode.h"
#include "SystemServices/Timer.h"
#include "TestMocks/MessageLogger.h"
#include "Messages/LocalWaypointChangeMsg.h"
#include "Messages/LocalConfigChangeMsg.h"
#include "Messages/CompassDataMsg.h"
#include <stdint.h>
#include <thread>

// For std::this_thread
#include <chrono>
#include <thread>

//I think these are needed
#define WAIT_FOR_MESSAGE		300
#define HTTP_TEST_COUNT				6

class HTTPSyncSuite : public CxxTest::TestSuite {
public:
	HTTPSyncNode* httpsync;
	std::thread* thr;
	MessageLogger* logger;
	int testCount = 0;
    DBHandler* dbhandler;
    DBLoggerNode* dbloggernode;
	MockNode* mockNode;
	bool nodeRegistered = false;

	//Timer timer;

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

    //TODO
    //Functions are private. Best to start thread and send messages to call syncing

	void setUp()
	{

        if (httpsync == 0)
        {
			mockNode = new MockNode(msgBus(), nodeRegistered);
            dbhandler = new DBHandler("../asr.db");
			dbloggernode = new DBLoggerNode(msgBus(),*dbhandler,100,1000,1);
			dbloggernode->start();
			logger = new MessageLogger(msgBus());
            httpsync = new HTTPSyncNode(msgBus(), dbhandler, 1, false);
			httpsync -> start();
            thr = new std::thread(runMessageLoop);
        }

        testCount++;
		std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_FOR_MESSAGE));

	}

    void tearDown()
	{
		if(testCount == HTTP_TEST_COUNT)
		{
			dbloggernode->stop();
			httpsync->stop();
			msgBus().stop();
			thr->join();
			delete thr;
			delete httpsync;
			delete dbloggernode;
			delete logger;
            delete dbhandler;
			delete mockNode;
			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		}
	}

    void test_HTTPSyncInit(){

        TS_ASSERT(httpsync->init());

    }

    void test_HTTPSyncValidURL(){

        std::string url1 = "http://www.sailingrobots.com/testdata/sync/";
        std::string url2 = "http://localhost/Remote-sailing-robots/sync/";
        std::string urlOrigin = dbhandler->retrieveCell("config_HTTPSyncNode", "1", "srv_addr");

        bool validCheck = (urlOrigin == url1 || urlOrigin == url2);

        TS_ASSERT(validCheck);

    }

    void test_HTTPSyncWaypoints(){

        std::string waypointString = dbhandler->getWaypoints();
        TS_TRACE("\nSending waypoint string: " + waypointString + "\n");
        if (waypointString == "")
        {
            TS_TRACE("Empty waypoint string - please run test with some waypoints in database");
        }
        else
        {
            TS_ASSERT(httpsync->pushWaypoints());
            std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_FOR_MESSAGE));

            httpsync->getWaypointsFromServer();
            std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_FOR_MESSAGE));

            std::string newWaypointString = dbhandler->getWaypoints();
            TS_TRACE("\nWaypoint string is now: " + newWaypointString + "\n");
            TS_TRACE("String comparison result: " + std::to_string(waypointString.compare(newWaypointString)));

            TS_ASSERT_EQUALS(waypointString.compare(newWaypointString),0);
        }


    }

    void test_HTTPSyncConfigs(){

        std::string configString = dbhandler->getConfigs();
        TS_ASSERT(httpsync->pushConfigs());
        std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_FOR_MESSAGE));

        httpsync->getConfigsFromServer();
        std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_FOR_MESSAGE));

        std::string newConfigString = dbhandler->getConfigs();

        TS_ASSERT_EQUALS(configString.compare(newConfigString),0);

    }

    void test_HTTPSyncPushDataLogs(){

		MessagePtr compassDatalogs = std::make_unique<CompassDataMsg>(3,2,1);
        msgBus().sendMessage(std::move(compassDatalogs));
        std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_FOR_MESSAGE));
        std::string currentLogs = dbhandler->getLogs(true);
        std::string emptyJson = "null";
        TS_ASSERT_DIFFERS(currentLogs.compare(emptyJson),0); //Logs are cleared on a successful push

		dbhandler->changeOneValue("httpsync_config","1","1","remove_logs");
		MessagePtr serverConfig = std::make_unique<ServerConfigsReceivedMsg>();
        msgBus().sendMessage(std::move(serverConfig));
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

		TS_ASSERT(httpsync->pushDatalogs());
		std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_FOR_MESSAGE));
		currentLogs = dbhandler->getLogs(true);
        TS_ASSERT_EQUALS(currentLogs.compare(emptyJson),0); //Logs are cleared on a successful push*/
	}

	void test_HTTPSyncUpdateDelay(){
		/* NOTE : Marc : Don't know gow to check it because there is no periodic value to check the frequence

		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		dbhandler->changeOneValue("httpsync_config","1","2","delay");
		/*MessagePtr serverConfig = std::make_unique<ServerConfigsReceivedMsg>();
		mockNode->m_MessageReceived = false;
        msgBus().sendMessage(std::move(serverConfig));
        std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_FOR_MESSAGE));
        while(not mockNode->m_MessageReceived);
        timer.start();
        mockNode->m_MessageReceived = false;
        while(not mockNode->m_MessageReceived);
        timer.stop();
        TS_ASSERT_DELTA(timer.timePassed(), 2, 1e-1);
	*/}
};
