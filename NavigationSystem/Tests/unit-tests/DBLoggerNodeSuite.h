
#pragma once

#include "Database/DBLoggerNode.h"
#include "MessageBus/MessageBus.h"
#include "MessageBusTestHelper.h"
#include "SystemServices/Logger.h"
#include "SystemServices/Timer.h"
#include "Tests/cxxtest/cxxtest/TestSuite.h"

#include <chrono>
#include <future>
#include <iostream>

#define DBLOGGERNODE_LOOP_TIME 1
#define DBLOGGERNODE_QUEUE_SIZE 1

class DBLoggerNodeSuite : public CxxTest::TestSuite {
   public:
    DBHandler* dbHandler;
    DBLoggerNode* dbLoggerNode;
    MessageBus messageBus;
    std::unique_ptr<MessageBusTestHelper> messageBusHelper;

    void setUp() {
        if (dbLoggerNode == 0) {
            dbHandler = new DBHandler("../asr.db");
            dbLoggerNode = new DBLoggerNode(messageBus, *dbHandler, DBLOGGERNODE_QUEUE_SIZE);
            messageBusHelper.reset(new MessageBusTestHelper(messageBus));
        }
    }

    void tearDown() {
        dbLoggerNode->stop();
        messageBusHelper.reset();
    }

    void test_LoggingToDB() {
        if (Logger::init()) {
            Logger::info("Built on %s at %s", __DATE__, __TIME__);
            Logger::info("Logger init\t\t[OK]");
        } else {
            Logger::error("Logger init\t\t[FAILED]");
        }

        if (dbHandler->initialise()) {
            Logger::info("Database init\t\t[OK]");
        } else {
            Logger::error("Database init\t\t[FAILED]");
            exit(1);
        }

        dbLoggerNode->start();

        Timer timer;

        timer.sleepUntil(DBLOGGERNODE_LOOP_TIME + 0.1);
    }
};
