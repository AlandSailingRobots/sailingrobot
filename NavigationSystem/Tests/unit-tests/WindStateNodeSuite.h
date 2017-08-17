#pragma once

#include "WorldState/WindStateNode.h"
#include "MessageBus/MessageBus.h"
#include "Tests/unit-tests/TestMocks/MessageLogger.h"
#include "Tests/unit-tests/TestMocks/MessageVerifier.h"
#include "Math/Utility.h"

#include "../cxxtest/cxxtest/TestSuite.h"

#include <chrono>
#include <thread>

#define WIND_STATE_TEST_COUNT 2

#define WIND_STATE_WAIT_TIME 750

class WindStateNodeSuite : public CxxTest::TestSuite {
public:

	WindStateNode* windStateNode;
  	std::thread* thr;
	MessageVerifier* verifier;
	MessageLogger* logger;

	float vesselHeading = 350;
    double vesselLat = 0;
    double vesselLon = 0;
	float vesselSpeed = 3;
	float vesselCourse = 0;

	float windSpeed = 4;
	float windDirection = 10;
	float windTemp =0;

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
			windStateNode = new WindStateNode(msgBus());
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

	void test_NodeSendsMessage()
	{
		msgBus().sendMessage(std::make_unique<WindDataMsg>(windDirection,windSpeed,windTemp));
		msgBus().sendMessage(std::make_unique<StateMessage>(vesselHeading,vesselLat,vesselLon,vesselSpeed,vesselCourse));

		std::this_thread::sleep_for(std::chrono::milliseconds(WIND_STATE_WAIT_TIME));
		TS_ASSERT(logger->windStateReceived());
	}

	void test_verifyCorrectMsgData()
	{
		msgBus().sendMessage(std::make_unique<WindDataMsg>(windDirection,windSpeed,windTemp));
		msgBus().sendMessage(std::make_unique<StateMessage>(vesselHeading,vesselLat,vesselLon,vesselSpeed,vesselCourse));
		std::this_thread::sleep_for(std::chrono::milliseconds(WIND_STATE_WAIT_TIME));

		TS_ASSERT(logger->windStateReceived());

		float apparentWindSpeed = 4;
		float apparentWindDirection = 10;

		float trueWindSpeed = 1;
		float trueWindDirection = 0;

		WindStateMsg windStateMsg(trueWindSpeed,trueWindDirection,apparentWindSpeed,apparentWindDirection);
	
		TS_ASSERT(verifier->verifyWindStateMsg(&windStateMsg));
	}
};