/****************************************************************************************
 *
 * File:
 * 		HTTPSyncSuite.h
 *
 * Purpose:
 *
 *
 * Developer Notes:
 *  - Database "asr.db" needs to be created and initialized before testing. Run:
 *      - ./installdb.sh  -  Server address needs to be
 *    http://sailingrobots.ax/aspire/sync/ or http://localhost/sync/
 *
 *  Mission needs to be Unit_tests.json
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

#include <stdint.h>
#include <thread>
#include "../cxxtest/cxxtest/TestSuite.h"
#include "Database/DBHandler.h"
#include "Database/DBLoggerNode.h"
#include "HTTPSync/HTTPSyncNode.h"
#include "MessageBus/MessageBus.h"
#include "Messages/CompassDataMsg.h"
#include "Messages/LocalConfigChangeMsg.h"
#include "Messages/LocalWaypointChangeMsg.h"
#include "SystemServices/Logger.h"
#include "SystemServices/Timer.h"
#include "TestMocks/MessageLogger.h"

// For std::this_thread
#include <chrono>
#include <thread>

#define HTTP_TEST_COUNT 6

class HTTPSyncSuite : public CxxTest::TestSuite {
   public:
    HTTPSyncNode* httpsync;
    std::thread* thr;
    MessageLogger* logger;

    DBHandler* dbhandler;
    DBLoggerNode* dbloggernode;
    MockNode* mockNode;

    bool nodeRegistered = false;
    const int WAIT_FOR_MESSAGE = 300;
    int testCount = 0;

    // Cheeky method for declaring and initialising a static in a header file
    static MessageBus& msgBus() {
        static MessageBus* mbus = new MessageBus();
        return *mbus;
    }

    static void runMessageLoop() { msgBus().run(); }

    // TODO
    // Functions are private. Best to start thread and send messages to call syncing

    void setUp() {
        if (httpsync == 0) {
            mockNode = new MockNode(msgBus(), nodeRegistered);
            dbhandler = new DBHandler("../asr.db");
            dbloggernode = new DBLoggerNode(msgBus(), *dbhandler, 1);
            dbloggernode->start();
            logger = new MessageLogger(msgBus());
            httpsync = new HTTPSyncNode(msgBus(), dbhandler);
            httpsync->start();
            thr = new std::thread(runMessageLoop);
        }

        testCount++;
        std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_FOR_MESSAGE));
    }

    void tearDown() {
        if (testCount == HTTP_TEST_COUNT) {
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
        }
    }

    void test_HTTPSyncInit() { TS_ASSERT(httpsync->init()); }

    void test_HTTPSyncValidURL() {
        std::string url1 = "http://sailingrobots.ax/aspire/sync/";
        std::string url2 = "http://localhost/sync/";
        std::string urlOrigin = dbhandler->retrieveCell("config_httpsync", "1", "srv_addr");

        bool validCheck = (urlOrigin == url1 || urlOrigin == url2);

        TS_ASSERT(validCheck);
    }

    void test_HTTPSyncWaypoints() {
        std::string waypointString = dbhandler->getWaypoints();
        TS_TRACE("\nSending waypoint string: " + waypointString + "\n");
        if (waypointString == "") {
            TS_TRACE("Empty waypoint string - please run test with some waypoints in database");
        } else {
            TS_ASSERT(httpsync->pushWaypoints());
            std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_FOR_MESSAGE));

            httpsync->getWaypointsFromServer();
            std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_FOR_MESSAGE));

            std::string newWaypointString = dbhandler->getWaypoints();
            TS_TRACE("\nWaypoint string is now: " + newWaypointString + "\n");
            TS_TRACE("String comparison result: " +
                     std::to_string(waypointString.compare(newWaypointString)));

            TS_ASSERT_EQUALS(waypointString.compare(newWaypointString), 0);
        }
    }

    void test_HTTPSyncConfigs() {
        std::string configString = dbhandler->getConfigs();
        TS_ASSERT(httpsync->pushConfigs());
        std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_FOR_MESSAGE));

        httpsync->getConfigsFromServer();
        std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_FOR_MESSAGE));

        std::string newConfigString = dbhandler->getConfigs();

        TS_ASSERT_EQUALS(configString.compare(newConfigString), 0);
    }

    void test_HTTPSyncPushDataLogs() {
        MessagePtr compassDatalogs = std::make_unique<CompassDataMsg>(3, 2, 1);
        msgBus().sendMessage(std::move(compassDatalogs));
        std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_FOR_MESSAGE));

        std::string currentLogs = dbhandler->getLogs(true);
        std::string emptyJson = "null";
        TS_ASSERT_DIFFERS(currentLogs.compare(emptyJson), 0);

        dbhandler->changeOneValue("config_httpsync", "1", "1", "remove_logs");
        MessagePtr serverConfig = std::make_unique<ServerConfigsReceivedMsg>();
        msgBus().sendMessage(std::move(serverConfig));
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        TS_ASSERT(httpsync->pushDatalogs());
    }
};
