#include "MessageBus/MessageBus.h"
#include "Tests/integration/TestMocks/MessagePrinter.h"
#include "Nodes/WindStateNode.h"
#include "Math/Utility.h"

#include <thread>
#include <chrono>

#define SLEEP_TIME 1000

MessageBus msgBus;

void msgBusLoop(){
	msgBus.run();
}

int main(int argc, char const *argv[])
{
	
	MessagePrinter printer(msgBus);
	WindStateNode WSNode(msgBus, 100);

	std::thread msgBusThread(msgBusLoop);
	msgBusThread.detach();

	MessagePtr windData = std::make_unique<WindDataMsg>(12,3,7);
	MessagePtr stateMsg = std::make_unique<StateMessage>(22,13,2,0.3);

	msgBus.sendMessage(std::move(windData));
	msgBus.sendMessage(std::move(stateMsg));
	msgBus.sendMessage(std::move(std::make_unique<StateMessage>(24,15,2,0.3)));

	std::this_thread::sleep_for(std::chrono::milliseconds(SLEEP_TIME));



	return 0;
}