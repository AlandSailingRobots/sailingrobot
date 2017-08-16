/****************************************************************************************
 *
 * File:
 * 		HardwareCompassSuite.h
 *
 * Purpose:
 *		A set of unit tests for checking whether the compass Node is functioning correctly
 *		These set of tests will only work if the compass is physically attached
 *
 * Developer Notes:
 *
 *							11.4.17 JM
 *
 *	Functions that have tests:		Functions that does not have tests:
 *
 *	init 							setOrientation
 *	start
 *	HMC6343ThreadFunc
 *	readData
 *
 ***************************************************************************************/

 #pragma once

 #include "../cxxtest/cxxtest/TestSuite.h"
#include "Hardwares/HMC6343Node.h"
#include "../../MessageBus/MessageBus.h"
#include "SystemServices/Logger.h"
#include "TestMocks/MessageLogger.h"
#include <stdint.h>
#include <thread>

// For std::this_thread
#include <chrono>
#include <thread>

#define WAIT_FOR_MESSAGE		300
#define COMPASS_TEST_COUNT				3


class HardwareCompassSuite : public CxxTest::TestSuite {
public:
    DBHandler* dbhandler;
	HMC6343Node* compass;
	std::thread* thr;
	MessageLogger* logger;
	int testCount = 0;

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
		if(compass == 0)
		{
			Logger::DisableLogging();
            dbhandler = new DBHandler("../asr.db");
			logger = new MessageLogger(msgBus());
			compass = new HMC6343Node(msgBus(),*dbhandler);
			thr = new std::thread(runMessageLoop);
		}
		testCount++;
		std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_FOR_MESSAGE));
	}

	void tearDown()
	{
		if(testCount == COMPASS_TEST_COUNT)
		{
			delete compass;
			delete logger;
		}
	}

	void test_CompassInit()
	{
		TS_ASSERT(compass->init());
	}

	void test_CompassRead()
	{
		float heading = 400;
		float pitch = 400;
		float roll = 400;

		TS_ASSERT(compass->readData(heading, pitch, roll));

		TS_ASSERT_LESS_THAN_EQUALS(heading, 360.f);
		TS_ASSERT_LESS_THAN_EQUALS(pitch, 90.f);
		TS_ASSERT_LESS_THAN_EQUALS(roll, 90.f);

		TS_ASSERT(heading >= 0.f);
		TS_ASSERT(pitch >= -90.f);
		TS_ASSERT(roll >= -90.f);
	}

	void test_CompassThread()
	{
		compass->start();

		std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_FOR_MESSAGE));

		TS_ASSERT(logger->compassDataReceived())
	}
};
