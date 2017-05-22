/****************************************************************************************
 *
 * File:
 * 		ActuatorRudderNodeSuite.h
 *
 * Purpose:
 *		A set of unit tests for checking whether the ActuatorRudderNodeSuite is working
 *      as intended
 *
 * Developer Notes:
 *
 *							11.4.17 JM
 *
 *	Functions that have tests:		Functions that does not have tests:
 *
 *	init
 *
 ***************************************************************************************/

 #pragma once

 #include "../cxxtest/cxxtest/TestSuite.h"
 #include "TestMocks/MessageLogger.h"
 #include "Nodes/ActuatorNode.h"
 #include "HardwareServices/MaestroController/MaestroController.h"

// For std::this_thread
#include <chrono>
#include <thread>

#define WAIT_FOR_MESSAGE		300
#define ACTUATOR_TEST_COUNT     1

 class ActuatorRudderNodeSuite : public CxxTest::TestSuite {
    public:
    ActuatorNode* rudder;
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
		if(rudder == 0)
		{
            Logger::DisableLogging();
			logger = new MessageLogger(msgBus());
            int channel = 3, speed = 0, acceleration = 0;
			rudder = new ActuatorNode(msgBus(), NodeID::RudderActuator, channel, speed, acceleration);
            thr = new std::thread(runMessageLoop);
		}
		testCount++;
        std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_FOR_MESSAGE));
	}

	void tearDown()
	{
		if(testCount == ACTUATOR_TEST_COUNT)
		{
			delete rudder;
			delete logger;
		}
	}

    void test_ActuatorRudderNodeInit()
    {
        MaestroController::init("/dev/ttyACM0");
        TS_ASSERT(rudder->init());
    }
 };
