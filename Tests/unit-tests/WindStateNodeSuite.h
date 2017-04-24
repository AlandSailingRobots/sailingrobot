#include "Nodes/WindStateNode.h"
#include "MessageBus/MessageBus.h"
#include "Tests/unit-tests/TestMocks/MessageLogger.h"
#include "Tests/unit-tests/TestMocks/MessageVerifier.h"
#include "Math/Utility.h"

#include "../cxxtest/cxxtest/TestSuite.h"

#include <chrono>
#include <thread>

#define WAIT_TIME 750

#define WIND_STATE_TEST_COUNT 2

class WindStateNodeSuite : public CxxTest::TestSuite {
public:

	WindStateNode* windStateNode;
  	std::thread* thr;
	MessageVerifier* verifier;
	MessageLogger* logger;

	float compassHeading = 33;
	double latitude = 48;
	double longitude = 13;
	double gpsSpeed = 34;
	double gpsCourse = 21;

	float windDir = 30;
	float windSpeed = 3;
	float windTemp = 90;
	int twdSize = 100;

	int testCount = 0;

	static MessageBus& msgBus(){
   	 	static MessageBus* mbus = new MessageBus();
    	return *mbus;
  	}

	static void runMessageLoop()
 	{
    	msgBus().run();
  	}

	void setUp() {
		if(windStateNode == 0){
			verifier = new MessageVerifier(msgBus());
			logger = new MessageLogger(msgBus());
			windStateNode = new WindStateNode(msgBus(), twdSize);
			thr = new std::thread(runMessageLoop);
		}
		testCount++;
	}

	void tearDown() {
		if(testCount == WIND_STATE_TEST_COUNT) {
			delete verifier;
			delete windStateNode;
		}
	}

	void test_NodeSendsMessage() {
	
		msgBus().sendMessage(std::make_unique<StateMessage>(compassHeading,latitude,longitude,gpsSpeed,gpsCourse));
		msgBus().sendMessage(std::make_unique<WindDataMsg>(windDir,windSpeed,windTemp));
		msgBus().sendMessage(std::make_unique<StateMessage>(compassHeading,latitude,longitude,gpsSpeed,gpsCourse));


		std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_TIME));
		TS_ASSERT(logger->windStateReceived());

	}

	void test_verifyCorrectMsgData() {


		msgBus().sendMessage(std::make_unique<StateMessage>(compassHeading,latitude,longitude,gpsSpeed,gpsCourse));
		msgBus().sendMessage(std::make_unique<WindDataMsg>(windDir,windSpeed,windTemp));
		msgBus().sendMessage(std::make_unique<StateMessage>(compassHeading,latitude,longitude,gpsSpeed,gpsCourse));
		std::this_thread::sleep_for(std::chrono::milliseconds(1500));

		TS_ASSERT(logger->windStateReceived());

		std::vector<float> twd;
		double trueWindSpeed = Utility::calculateTrueWindSpeed(windDir, windSpeed, gpsSpeed, compassHeading);
		double trueWindDirection = Utility::getTrueWindDirection(windDir, windSpeed, gpsSpeed, compassHeading, twd, twdSize);

		double apparentWindSpeed;
		double apparentWindDirection;

		Utility::calculateApparentWind(windDir,windSpeed,gpsSpeed,
			compassHeading,trueWindDirection,apparentWindSpeed,apparentWindDirection);

		WindStateMsg windStateMsg(trueWindSpeed,trueWindDirection,apparentWindSpeed,apparentWindDirection);
	
		TS_ASSERT(verifier->verifyWindStateMsg(&windStateMsg));
	}

};