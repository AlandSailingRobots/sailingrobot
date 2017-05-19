
#pragma once

#include "SystemServices/Logger.h"
#include "dbhandler/DBHandler.h"
#include "dbhandler/DBLogger.h"
#include "../cxxtest/cxxtest/TestSuite.h"

#include <iostream>
#include <thread>
#include <chrono>
#include <future>

#define DBLOGGER_WAIT_TIME 2

class DBLoggerNodeSuite : public CxxTest::TestSuite {

    public:

    void setUp() {}
    void tearDown() {}

    void test_LoggingToDB() {
        DBHandler dbHandler("./asrtest.db");

        if (Logger::init())
        {
		    Logger::info("Built on %s at %s", __DATE__, __TIME__);
		    Logger::info("Logger init\t\t[OK]");
	    }
	    else
        {
		    Logger::error("Logger init\t\t[FAILED]");
	    }

        if(dbHandler.initialise()) 
        {
		    Logger::info("Database init\t\t[OK]");
	    }
	    else 
        {
		    Logger::error("Database init\t\t[FAILED]");
		    Logger::shutdown();
		    exit(1);
	    }

        DBLogger dbLogger(5, dbHandler);
        dbLogger.startWorkerThread();
        TS_ASSERT(true);
        dbLogger.log(NULL, 1, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, true, "hejhej");
        dbLogger.log(NULL, 1, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, true, "hejhej2");
        dbLogger.log(NULL, 1, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, true, "hejhej3");
        dbLogger.log(NULL, 1, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, true, "hejhej4");
        dbLogger.log(NULL, 1, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, true, "hejhej5");
        std::this_thread::sleep_for(std::chrono::seconds(DBLOGGER_WAIT_TIME));
    }
};