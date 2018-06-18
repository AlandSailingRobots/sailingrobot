/****************************************************************************************
 *
 * File:
 * 		HardwareCV7Suite.h
 *
 * Purpose:
 *		A set of unit tests for checking whether the CV7 Node is functioning correctly
 *		These set of tests will only work if the wind sensor is physically attached
 *
 * Developer Notes:
 *
 *							11.4.17 JM
 *
 *	Functions that have tests:		Functions that does not have tests:
 *
 *	init
 *	start
 *	parseString
 *	WindSensorThread
 *
 ***************************************************************************************/

#pragma once

#include <stdint.h>
#include <thread>
#include "../cxxtest/cxxtest/TestSuite.h"
#include "DataBase/DBHandler.h"
#include "Hardwares/CV7Node.h"
#include "MessageBus/MessageBus.h"
#include "SystemServices/Logger.h"
#include "TestMocks/MessageLogger.h"

// For std::this_thread
#include <chrono>
#include <thread>

#define WAIT_FOR_MESSAGE 300
#define WINDSENSOR_TEST_COUNT 4

class HardwareCV7Suite : public CxxTest::TestSuite {
   public:
    CV7Node* cv7;
    std::thread* thr;
    MessageLogger* logger;
    int testCount = 0;

    // Cheeky method for declaring and initialising a static in a header file
    static MessageBus& msgBus() {
        static MessageBus* mbus = new MessageBus();
        return *mbus;
    }

    static void runMessageLoop() { msgBus().run(); }

    void setUp() {
        // Only want to setup them up once in this test, only going to delete them when the program
        // closes and the OS destroys the process's memory
        if (cv7 == 0) {
            DBHandler dbHandler("./asr.db");

            Logger::DisableLogging();
            logger = new MessageLogger(msgBus());
            // cv7 = new CV7Node(msgBus(), dbHandler.retrieveCell("windsensor_config", "1", "port"),
            // dbHandler.retrieveCellAsInt("windsensor_config", "1", "baud_rate")); // Not in the
            // database
            thr = new std::thread(runMessageLoop);
        }
        testCount++;
        std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_FOR_MESSAGE));
    }

    void tearDown() {
        if (testCount == WINDSENSOR_TEST_COUNT) {
            delete cv7;
            delete logger;
        }
    }

    void test_CV7Init() { TS_ASSERT(cv7->init()); }

    void test_CV7ParseSuccesfully() {
        std::string sensorData = "$IIMWV,125.8,R,015.8,N,A*3F$WIXDR,C,036.5,C,,*52";
        float windDir = 0;
        float windSpeed = 0;
        float windTemp = 0;

        TS_ASSERT(cv7->parseString(sensorData, windDir, windSpeed, windTemp));

        TS_ASSERT_EQUALS(windDir, 125.8f);
        TS_ASSERT_EQUALS(windSpeed, 15.8f);
        TS_ASSERT_EQUALS(windTemp, 36.5f);

        // TODO - Jordan: Add a test for the temperature data
    }

    void test_CV7ParseFail() {
        std::string sensorData = "";
        float windDir = 0;
        float windSpeed = 0;
        float windTemp = 0;

        TS_ASSERT(not cv7->parseString(sensorData, windDir, windSpeed, windTemp));
    }

    void test_CV7Thread() {
        cv7->start();

        std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_FOR_MESSAGE));

        TS_ASSERT(logger->windDataReceived());
    }
};
