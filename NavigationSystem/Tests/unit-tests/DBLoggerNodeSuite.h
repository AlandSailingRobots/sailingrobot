
#pragma once

#include "SystemServices/Timer.h"
#include "SystemServices/Logger.h"
#include "MessageBus/MessageBus.h"
#include "DataBase/DBLoggerNode.h"
#include "MessageBusTestHelper.h"
#include "../cxxtest/cxxtest/TestSuite.h"

#include <iostream>
#include <chrono>
#include <future>

#define DBLOGGERNODE_LOOP_TIME 1
#define DBLOGGERNODE_QUEUE_SIZE 1

class DBLoggerNodeSuite : public CxxTest::TestSuite {

    public:

    DBHandler* dbHandler;
    DBLoggerNode* dbLoggerNode;
    MessageBus messageBus;
    std::unique_ptr<MessageBusTestHelper> messageBusHelper;

    void setUp() {
        if(dbLoggerNode == 0) {
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

        if (Logger::init())
        {
		    Logger::info("Built on %s at %s", __DATE__, __TIME__);
		    Logger::info("Logger init\t\t[OK]");
	    }
	    else
        {
		    Logger::error("Logger init\t\t[FAILED]");
	    }

        if(dbHandler->initialise())
        {
		    Logger::info("Database init\t\t[OK]");
	    }
	    else
        {
		    Logger::error("Database init\t\t[FAILED]");
		    Logger::shutdown();
		    exit(1);
	    }

        dbLoggerNode->start();

        Timer timer;

        timer.sleepUntil(DBLOGGERNODE_LOOP_TIME + 0.1);

    }
};
