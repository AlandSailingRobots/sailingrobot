
#pragma once

#include "Hardwares/ActuatorNodeASPire.h"

#include "MessageBus/MessageBus.h"
#include "MessageBus/Message.h"
#include "Messages/RudderCommandMsg.h"
#include "Hardwares/CAN_Services/CANService.h"

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
        MessagePtr msg = std::make_unique<RudderCommandMsg>(20);
        // Listen to PGN 700
        std::vector<uint32_t> PGNs = { 700 };
        MockCANReceiver mockReceiver(CANService, PGNs);
        std::thread t1(runMessageLoop);
        t1.detach();
        auto fu = CANService.start();
        CANService.SetLoopBackMode();

        msgBus().sendMessage(std::move(msg));

        std::this_thread::sleep_for(std::chrono::milliseconds(750));

        TS_ASSERT(mockReceiver.message_received());

        CANService.stop();
        fu.get();
    }

};