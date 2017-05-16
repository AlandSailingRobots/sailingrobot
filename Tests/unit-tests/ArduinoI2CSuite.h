/****************************************************************************************
 *
 * File:
 * 		ArduinoI2CSuite.h
 *
 * Purpose:
 *
 * License:
 *      
 *
 * Developer Notes:
 *
 *							12.4.17 JM
 *
 *	Unable to find ArduinoI2CNode.h in Nodes
 *
 ***************************************************************************************/

#pragma once


#include "../cxxtest/cxxtest/TestSuite.h"
#include "../../MessageBus/MessageBus.h"
#include "TestMocks/MessageLogger.h"
#include "Nodes/ArduinoI2CNode.h"
#include "SystemServices/Logger.h"

// For std::this_thread
#include <chrono>
#include <thread>

#define WAIT_FOR_MESSAGE		300
#define ARDUINO_TEST_COUNT     	1

class ArduinoI2CSuite : public CxxTest::TestSuite {
public:
    ArduinoI2CNode* ard;
    MessageLogger* logger;
    std::thread* thr;
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
		if(ard == 0)
		{
            Logger::DisableLogging();
			logger = new MessageLogger(msgBus());
			ard = new ArduinoI2CNode(msgBus());
            thr = new std::thread(runMessageLoop);
		}
		testCount++;
        std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_FOR_MESSAGE));
	}

	void tearDown()
	{
		if(testCount == ARDUINO_TEST_COUNT)
		{
			delete ard;
			delete logger;
		}
	}
	
	
	
	void test_init()
	{
		TS_ASSERT(ard->init());
	}
};
