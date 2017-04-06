/****************************************************************************************
 *
 * File:
 * 		MaestroControllerSuite.h
 *
 * Purpose:
 *		A set of unit tests for checking whether the MaestroController is working and
 *      connected
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/

 #pragma once

 #include "../cxxtest/cxxtest/TestSuite.h"
 #include "HardwareServices/MaestroController/MaestroController.h"

// For std::this_thread
#include <chrono>
#include <thread>

#define WAIT_FOR_MESSAGE		300
#define MAESTRO_TEST_COUNT     	2

 class MaestroControllerSuite : public CxxTest::TestSuite {
    public:
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
        int i = 0;
		if(i == 0)
		{
            Logger::DisableLogging();
            thr = new std::thread(runMessageLoop);
		}
		testCount++;
        std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_FOR_MESSAGE));
	}

	void tearDown()
	{
		if(testCount == MAESTRO_TEST_COUNT)
		{
		}
	}

    void test_MaestroControllerInit()
    {
        TS_ASSERT(MaestroController::init("/dev/ttyACM0"));
    }

    void test_MaestroControllerWriteCommand()
    {
        TS_ASSERT(MaestroController::writeCommand(MaestroCommands::SetPosition, 3, 4500));
    }

    void test_MaestroControllerGetError()
    {
        TS_ASSERT_EQUALS(MaestroController::getError(), 0);
    }
 };
