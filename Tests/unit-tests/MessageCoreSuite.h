/****************************************************************************************
 *
 * File:
 * 		MessageCoreSuite.h
 *
 * Purpose:
 *		A set of unit tests for ensuring messages flow through the system and tests for 
 *		core framework.
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/


 #pragma once

#include "../cxxtest/cxxtest/TestSuite.h"
#include "TestMocks/MockNode.h"
#include "../../MessageBus/MessageBus.h"
#include "SystemServices/Logger.h"
#include <stdint.h>
#include <thread>

// For std::this_thread
#include <chrono>
#include <thread>

#define WAIT_FOR_MESSAGE		300


class MessageCoreSuite : public CxxTest::TestSuite {
public:
	MockNode* node;
	std::thread* thr;
	bool registered;

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
		if(node == 0)
		{
			Logger::DisableLogging();
			registered = false;
			node = new MockNode(msgBus(), registered);
			thr = new std::thread(runMessageLoop);
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_FOR_MESSAGE));
	}

	void tearDown()
	{
		//delete node;
	}

	void test_NodeRegisteredAtStart()
	{
		// Should of been set to true in the constructor of MockNode
		TS_ASSERT(registered);
	}

	void test_FailedNodeRegisterAfterStart()
	{
		bool didRegister = false;
		MockNode nodeTwo(msgBus(), didRegister);

		TS_ASSERT(!didRegister);
	}

	void test_MessageReceived()
	{
		TS_TRACE("START");
		MessagePtr windData = std::make_unique<WindDataMsg>(0,0,0);
		MessagePtr bluffData = std::make_unique<WindDataMsg>(0,0,0);
		TS_TRACE("POINT A");
		msgBus().sendMessage(std::move(bluffData));
		TS_TRACE("POINT B");
		msgBus().sendMessage(std::move(windData));

		// Wait for the message to go through
		std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_FOR_MESSAGE));
		bool response = node->m_MessageReceived;
		node->m_MessageReceived = false;

		TS_ASSERT(response);

		//delete windData;
	}

	void test_DirectMessage()
	{

		MessagePtr windData = std::make_unique<WindDataMsg>(NodeID::MessageLogger, NodeID::None, 0, 0, 0);

		msgBus().sendMessage(std::move(windData));

		// Wait for the message to go through
		std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_FOR_MESSAGE));

		bool response = node->m_MessageReceived;
		node->m_MessageReceived = false;

		TS_ASSERT(response); 
	}

	void test_DirectMessageToSomeoneElse()
	{
		MessagePtr windData = std::make_unique<WindDataMsg>(NodeID::Compass, NodeID::None, 0, 0, 0);

		msgBus().sendMessage(std::move(windData));

		// Wait for the message to go through
		std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_FOR_MESSAGE));

		bool response = node->m_MessageReceived;
		node->m_MessageReceived = false;

		TS_ASSERT(not response);
	}

	void test_MessageWithDataReceived()
	{
		MessagePtr windData = std::make_unique<WindDataMsg>(WindDataMsg(NodeID::MessageLogger, NodeID::None, 120, 90, 60));

		msgBus().sendMessage(std::move(windData));

		// Wait for the message to go through
		std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_FOR_MESSAGE));

		TS_ASSERT_EQUALS(node->m_WindDir, 120);
		TS_ASSERT_EQUALS(node->m_WindSpeed, 90);
		TS_ASSERT_EQUALS(node->m_WindTemp, 60);
	}
};



