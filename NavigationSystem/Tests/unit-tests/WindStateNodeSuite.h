#pragma once

#include "Math/Utility.h"
#include "MessageBus/MessageBus.h"
#include "MessageBusTestHelper.h"
#include "Tests/unit-tests/TestMocks/MessageLogger.h"
#include "Tests/unit-tests/TestMocks/MessageVerifier.h"
#include "WorldState/WindStateNode.h"

#include "../cxxtest/cxxtest/TestSuite.h"

#include <chrono>

#define WIND_STATE_TEST_COUNT 2

#define WIND_STATE_WAIT_TIME 750

class WindStateNodeSuite : public CxxTest::TestSuite {
   public:
    WindStateNode* windStateNode;
    DBHandler* dbhandler;
    MessageVerifier* verifier;
    MessageLogger* logger;

    float vesselHeading = 350;
    double vesselLat = 0;
    double vesselLon = 0;
    float vesselSpeed = 3;
    float vesselCourse = 0;

    float windSpeed = 4;
    float windDirection = 10;
    float windTemp = 0;

    int testCount = 0;
    MessageBus messageBus;
    std::unique_ptr<MessageBusTestHelper> messageBusHelper;
    void setUp() {
        if (windStateNode == 0) {
            verifier = new MessageVerifier(messageBus);
            dbhandler = new DBHandler("../asr.db");
            logger = new MessageLogger(messageBus);
            windStateNode = new WindStateNode(messageBus);
            messageBusHelper.reset(new MessageBusTestHelper(messageBus));
        }
        testCount++;
    }

    void tearDown() {
        if (testCount == WIND_STATE_TEST_COUNT) {
            messageBusHelper.reset();
            delete logger;
            delete verifier;
            delete windStateNode;
            delete dbhandler;
        }
    }

    void test_NodeSendsMessage() {
        messageBus.sendMessage(std::make_unique<WindDataMsg>(windDirection, windSpeed, windTemp));
        messageBus.sendMessage(std::make_unique<StateMessage>(vesselHeading, vesselLat, vesselLon,
                                                              vesselSpeed, vesselCourse));

        std::this_thread::sleep_for(std::chrono::milliseconds(WIND_STATE_WAIT_TIME));
        TS_ASSERT(logger->windStateReceived());
    }

    void test_verifyCorrectMsgData() {
        TS_SKIP("Test skipped for now, we need correct values to test with");
        /*
        messageBus.sendMessage(std::make_unique<WindDataMsg>(windDirection, windSpeed, windTemp));
        messageBus.sendMessage(std::make_unique<StateMessage>(vesselHeading, vesselLat, vesselLon,
                                                              vesselSpeed, vesselCourse));
        std::this_thread::sleep_for(std::chrono::milliseconds(WIND_STATE_WAIT_TIME));

        TS_ASSERT(logger->windStateReceived());

        float apparentWindSpeed = 4;
        float apparentWindDirection = 10;

        float trueWindSpeed = 1;
        float trueWindDirection = 0;

        WindStateMsg windStateMsg(trueWindSpeed, trueWindDirection, apparentWindSpeed,
                                  apparentWindDirection);

        TS_ASSERT(verifier->verifyWindStateMsg(&windStateMsg));*/
    }
};
