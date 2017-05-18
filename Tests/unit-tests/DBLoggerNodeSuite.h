
#pragma once

#include "SystemServices/Logger.h"
#include "dbhandler/DBLogger.h"
#include "../cxxtest/cxxtest/TestSuite.h"

#include <thread>
#include <iostream>

class DBLoggerNodeSuite : public CxxTest::TestSuite {
    public:

    void setUp() {}
    void tearDown() {}

    void test_LoggingToDB() {
        DBHandler dbHandler("./asrtest.db");

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
    }
};