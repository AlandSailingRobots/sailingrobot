
#pragma once

#include "Nodes/ActuatorNodeASPire.h"

#include "MessageBus/MessageBus.h"
#include "Messages/Message.h"
#include "Messages/ActuatorControlASPireMessage.h"
#include "HardwareServices/CAN_Services/CANService.h"

#include "../cxxtest/cxxtest/TestSuite.h"
#include "TestMocks/MockCANReceiver.h"

#include <thread>
#include <chrono>

class ActuatorNodeASPireSuite : public CxxTest::TestSuite {
public:

    static MessageBus& msgBus()
	{
		static MessageBus* mbus = new MessageBus();
		return *mbus;
	}

	static void runMessageLoop()
	{
		msgBus().run();
	}

    void setUp() {}
    void tearDown() {}

    void test_NodeSendsFrame() {
        CANService CANService;

        ActuatorNodeASPire node(msgBus(), CANService);
        MessagePtr msg = std::make_unique<ActuatorControlASPireMessage>(2.3, 1.5, false);
        // Listen to PGN 700
        std::vector<uint32_t> PGNs = { 700 };
        MockCANReceiver mockReceiver(CANService, PGNs);
        std::thread t1(runMessageLoop);
        auto fu = CANService.start();

        msgBus().sendMessage(std::move(msg));

        std::this_thread::sleep_for(std::chrono::milliseconds(750));

        TS_ASSERT(mockReceiver.message_received());

        CANService.stop();
        fu.get();
    }

};