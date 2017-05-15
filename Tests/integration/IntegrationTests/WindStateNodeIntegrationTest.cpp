#include "MessageBus/MessageBus.h"
#include "Tests/integration/TestMocks/MessagePrinter.h"
#include "Tests/unit-tests/TestMocks/MessageVerifier.h"
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
	MessageVerifier verifier(msgBus);
	WindStateNode WSNode(msgBus, 100);

	std::thread msgBusThread(msgBusLoop);
	msgBusThread.detach();

	const int twdSize = 100;
	WindStateNode windStateNode(msgBus, twdSize);

	float compassHeading = 33;
	double latitude = 48;
	double longitude = 13;
	double gpsSpeed = 34;
	double gpsCourse = 21;

	float windDir = 30;
	float windSpeed = 3;
	float windTemp = 90;

	MessagePtr windData = std::make_unique<WindDataMsg>(windDir,windSpeed,windTemp);
	MessagePtr stateMsg = std::make_unique<StateMessage>(compassHeading,latitude,longitude,gpsSpeed,gpsCourse);

	msgBus.sendMessage(std::make_unique<StateMessage>(compassHeading,latitude,longitude,gpsSpeed,gpsCourse));
	msgBus.sendMessage(std::make_unique<WindDataMsg>(windDir,windSpeed,windTemp));
	msgBus.sendMessage(std::make_unique<StateMessage>(compassHeading,latitude,longitude,gpsSpeed,gpsCourse));
	
	std::this_thread::sleep_for(std::chrono::milliseconds(1500));

	std::vector<float> twd;
	double trueWindDirection = Utility::getTrueWindDirection(windDir, windSpeed, gpsSpeed, compassHeading, twd, twdSize);
	double trueWindSpeed =	   Utility::calculateTrueWindSpeed(windDir, windSpeed, gpsSpeed, compassHeading);

	double apparentWindSpeed;
	double apparentWindDirection;

	Utility::calculateApparentWind(windDir,windSpeed,gpsSpeed,
		compassHeading,trueWindDirection,apparentWindSpeed,apparentWindDirection);

	WindStateMsg windStateMsg(trueWindSpeed,trueWindDirection,apparentWindSpeed,apparentWindDirection);

	std::cout << verifier.verifyWindStateMsg(&windStateMsg) << std::endl;

	std::this_thread::sleep_for(std::chrono::milliseconds(1500));

	return 0;
}