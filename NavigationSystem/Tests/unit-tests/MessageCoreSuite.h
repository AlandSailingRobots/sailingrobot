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
 *							12.4.17 JM
 *
 *	Functions that have tests:		Functions that does not have tests:
 *
 *	sendMessage						registerNode
 *	run 							getRegisteredNode
 *									processMessages
 *									startMessageLog
 *									logMessageReceived
 *									logMessage
 *									logMessageConsumer
 *									messageTimeStamp
 *
 ***************************************************************************************/

#pragma once

#include <stdint.h>
#include "../../MessageBus/MessageBus.h"
#include "MessageBusTestHelper.h"
#include "../../SystemServices/Logger.h"
#include "../cxxtest/cxxtest/TestSuite.h"
#include "TestMocks/MockNode.h"

// For std::this_thread
#include <chrono>

#define MESSAGE_CORE_TESTCOUNT 6

class MessageCoreSuite : public CxxTest::TestSuite {
   public:
    MockNode* node;
    std::thread* thr;
    bool registered;
    int testcount = 0;
    const int WAIT_FOR_MESSAGE = 300;
    MessageBus messageBus;
    std::unique_ptr<MessageBusTestHelper> messageBusHelper;

    // Cheeky method for declaring and initialising a static in a header file
    /*
    static MessageBus& msgBus()
    {
        static MessageBus* mbus = new MessageBus();
        return *mbus;
    }

    static void runMessageLoop()
    {
        msgBus().run();
    }
*/
    void setUp() {
        // Only want to setup them up once in this test, only going to delete them when the program
        // closes and the OS destroys the process's memory
        if (node == 0) {
            Logger::DisableLogging();
            registered = false;
            node = new MockNode(messageBus, registered);
            /*
            thr = new std::thread(runMessageLoop);*/
            messageBusHelper.reset(new MessageBusTestHelper(messageBus));
        }
        testcount++;
        std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_FOR_MESSAGE));
    }

    void tearDown() {
        if (testcount == MESSAGE_CORE_TESTCOUNT) {
            messageBusHelper.reset();
            /*
            msgBus().stop();
            thr->join();
            delete thr;*/
            delete node;
        }
    }

    void test_NodeRegisteredAtStart() {
        // Should of been set to true in the constructor of MockNode
        TS_ASSERT(registered);
    }

    void test_FailedNodeRegisterAfterStart() {
        bool didRegister = false;
        MockNode nodeTwo(messageBus, didRegister);

        TS_ASSERT(!didRegister);
    }

    void test_MessageReceived() {
        TS_TRACE("START");
        MessagePtr windData = std::make_unique<WindDataMsg>(0, 0, 0);
        MessagePtr bluffData = std::make_unique<WindDataMsg>(0, 0, 0);
        TS_TRACE("POINT A");
        messageBus.sendMessage(std::move(bluffData));
        TS_TRACE("POINT B");
        messageBus.sendMessage(std::move(windData));

        // Wait for the message to go through
        std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_FOR_MESSAGE));
        bool response = node->m_MessageReceived;
        node->m_MessageReceived = false;

        TS_ASSERT(response);

        // delete windData;
    }

    void test_DirectMessage() {
        MessagePtr windData =
            std::make_unique<WindDataMsg>(NodeID::MessageLogger, NodeID::None, 0, 0, 0);

        messageBus.sendMessage(std::move(windData));

        // Wait for the message to go through
        std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_FOR_MESSAGE));

        bool response = node->m_MessageReceived;
        node->m_MessageReceived = false;

        TS_ASSERT(response);
    }

    void test_DirectMessageToSomeoneElse() {
        MessagePtr windData = std::make_unique<WindDataMsg>(NodeID::Compass, NodeID::None, 0, 0, 0);

        messageBus.sendMessage(std::move(windData));

        // Wait for the message to go through
        std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_FOR_MESSAGE));

        bool response = node->m_MessageReceived;
        node->m_MessageReceived = false;

        TS_ASSERT(not response);
    }

    void test_MessageWithDataReceived() {
        MessagePtr windData = std::make_unique<WindDataMsg>(
            WindDataMsg(NodeID::MessageLogger, NodeID::None, 120, 90, 60));

        messageBus.sendMessage(std::move(windData));
        // Wait for the message to go through
        std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_FOR_MESSAGE));

        TS_ASSERT_EQUALS(node->m_WindDir, 120);
        TS_ASSERT_EQUALS(node->m_WindSpeed, 90);
        TS_ASSERT_EQUALS(node->m_WindTemp, 60);
    }
};
