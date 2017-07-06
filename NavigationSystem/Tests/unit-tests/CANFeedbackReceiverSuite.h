#pragma once

#include "Hardwares/CAN_Services/N2kMsg.h"
#include "Hardwares/CAN_Services/CANService.h"

#include "Hardwares/CANFeedbackReceiver.h"
#include "Tests/unit-tests/TestMocks/MessageLogger.h"
#include "Tests/unit-tests/TestMocks/MessageVerifier.h"
#include "MessageBus/MessageBus.h"

#include "../cxxtest/cxxtest/TestSuite.h"

#include <thread>
#include <chrono>

#define CAN_FR_TESTCOUNT 1

class CANFeedbackReceiverSuite : public CxxTest::TestSuite {
public:
    int testCount = 0;
    CANFeedbackReceiver* receiver;
  	std::thread* thr;
	MessageVerifier* verifier;


	static MessageBus& msgBus(){
   	 	static MessageBus* mbus = new MessageBus();
    	return *mbus;
  	}

    static CANService& canService(){
        static CANService* service = new CANService();
        return *service;
    }

	static void runMessageLoop()
 	{
    	msgBus().run();

  	}

	void setUp() {
		if(receiver == 0){
            receiver = new CANFeedbackReceiver(msgBus(),canService(),1);
			verifier = new MessageVerifier(msgBus());
			thr = new std::thread(runMessageLoop);
            canService().start();
		}
		testCount++;
	}

	void tearDown() {
		if(testCount == CAN_FR_TESTCOUNT) {
			delete verifier;
			delete receiver;
		}
	}

    void test_NodeSendsMessageOnFrameReceive() {
        float ratio = 65535 / 60;
        uint16_t rudderFeedback = 21 * ratio;
        uint16_t wingsailFeedback = 13 * ratio;
        uint16_t windvaneSteerAngle = 2 * ratio;
        CanMsg Cmsg;
        Cmsg.id = 701;
        Cmsg.header.ide = 0;
        Cmsg.header.length = 7;

        (Cmsg.data[0] = rudderFeedback & 0xff);
        (Cmsg.data[1] = rudderFeedback >> 8);
        (Cmsg.data[2] = wingsailFeedback & 0xff);
        (Cmsg.data[3] = wingsailFeedback >> 8);
        (Cmsg.data[4] = windvaneSteerAngle & 0xff);
        (Cmsg.data[5] = windvaneSteerAngle >> 8);
        (Cmsg.data[6] = 0);

        canService().sendCANMessage(Cmsg);

        ASPireActuatorFeedbackMsg otherMsg(rudderFeedback, wingsailFeedback, windvaneSteerAngle, 0);

        std::this_thread::sleep_for(std::chrono::milliseconds(750));
        TS_ASSERT(verifier->verifyActuatorFeedbackMsg(&otherMsg))

    }

};
