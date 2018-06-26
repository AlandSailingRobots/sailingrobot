#pragma once

#include "../Hardwares/CAN_Services/CANService.h"
#include "../Hardwares/CAN_Services/CanBusCommon/CanMessageHandler.h"
#include "../Hardwares/CAN_Services/N2kMsg.h"

#include "../Database/DBHandler.h"
#include "../Hardwares/CANArduinoNode.h"
#include "../MessageBus/MessageBus.h"
#include "../Tests/unit-tests/TestMocks/MessageLogger.h"
#include "../Tests/unit-tests/TestMocks/MessageVerifier.h"

#include "../cxxtest/cxxtest/TestSuite.h"

#include <chrono>
#include <thread>

#define CAN_FR_TESTCOUNT 1

class CANArduinoNodeSuite : public CxxTest::TestSuite {
   public:
    int testCount = 0;
    CANArduinoNode* receiver;
    std::thread* thr;
    MessageVerifier* verifier;
    DBHandler* dbhandler;

    static MessageBus& msgBus() {
        static MessageBus* mbus = new MessageBus();
        return *mbus;
    }

    static CANService& canService() {
        static CANService* service = new CANService();
        return *service;
    }

    static void runMessageLoop() { msgBus().run(); }

    void setUp() {
        if (receiver == 0) {
            receiver = new CANArduinoNode(msgBus(), *dbhandler, canService());
            verifier = new MessageVerifier(msgBus());
            thr = new std::thread(runMessageLoop);
            canService().start();
        }
        testCount++;
    }

    void tearDown() {
        if (testCount == CAN_FR_TESTCOUNT) {
            delete verifier;
            delete receiver;
        }
    }

    void test_NodeSendsMessageOnFrameReceive() {
        float ratio = 65535 / 60;
        uint16_t rudderFeedback = 21 * ratio;
        uint16_t wingsailFeedback = 13 * ratio;
        uint16_t windvaneSteerAngle = 2 * ratio;

        CanMessageHandler messageHandler(MSG_ID_AU_FEEDBACK);
        messageHandler.encodeMessage(RUDDER_ANGLE_DATASIZE, rudderFeedback);
        messageHandler.encodeMessage(WINGSAIL_ANGLE_DATASIZE, wingsailFeedback);
        messageHandler.encodeMessage(WINDVANE_SELFSTEERING_DATASIZE, windvaneSteerAngle);

        CanMsg Cmsg = messageHandler.getMessage();
        canService().sendCANMessage(Cmsg);

        ASPireActuatorFeedbackMsg otherMsg(wingsailFeedback, rudderFeedback, windvaneSteerAngle, 0,
                                           0);

        std::this_thread::sleep_for(std::chrono::milliseconds(750));
        TS_ASSERT(verifier->verifyActuatorFeedbackMsg(&otherMsg))
    }
};
