#include "Nodes/WindStateNode.h"
#include "MessageBus/MessageBus.h"
#include "Tests/unit-tests/TestMocks/MessageLogger.h"
#include "../cxxtest/cxxtest/TestSuite.h"

#include <chrono>
#include <thread>

#define WAIT_TIME 300


class WindStateNodeSuite : public CxxTest::TestSuite {
public:

	std::thread* msgBusThread;

	static MessageBus& msgBus(){
   		static MessageBus* mbus = new MessageBus();
    	return *mbus;
  	}

  	static void messageBusLoop(){
  		

  		msgBus().run();
  	}

	void setUp() {
		msgBusThread = new std::thread(messageBusLoop);
	}

	void tearDown() {

	}

	void test_NodeSendsMessage() {
		MessageLogger logger(msgBus());
		WindStateNode windStateNode(msgBus(), 100);

		MessagePtr windData = std::make_unique<WindDataMsg>(1,2,3);
		MessagePtr stateMsg = std::make_unique<StateMessage>(4,5,6,7);

		msgBus().sendMessage(std::move(windData));
		msgBus().sendMessage(std::move(stateMsg));


		std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_TIME));
		TS_ASSERT(logger.windStateReceived());

	}

};