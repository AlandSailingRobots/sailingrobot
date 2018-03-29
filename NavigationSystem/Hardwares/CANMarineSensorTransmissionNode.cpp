//
// Created by dkarlsso on 3/29/18.
//

#include "CANMarineSensorTransmissionNode.h"
#include "../MessageBus/NodeIDs.h"
#include "../MessageBus/Message.h"
#include "../MessageBus/MessageTypes.h"
#include "../SystemServices/Timer.h"
#include "../MessageBus/MessageBus.h"
#include "../Messages/MarineSensorDataRequestMsg.h"


#include "MessageBus/NodeIDs.h"
#include "MessageBus/Message.h"
#include "MessageBus/MessageTypes.h"
#include "SystemServices/Timer.h"
#include "MessageBus/MessageBus.h"
#include "Messages/MarineSensorDataRequestMsg.h"

CANMarineSensorTransmissionNode::CANMarineSensorTransmissionNode(MessageBus &msgBus, DBHandler &dbhandler,
                                                                 CANService &canService) : ActiveNode(NodeID::MarineSensor, msgBus),
                                                                                           m_CANService(canService), m_LoopTime(0.5) {
    msgBus.registerNode(*this, MessageType::DataRequest);
}

CANMarineSensorTransmissionNode::~CANMarineSensorTransmissionNode() {}

void CANMarineSensorTransmissionNode::processMessage(const Message* message) {
    if(message->messageType() == MessageType::DataRequest){
        //MessagePtr marineSensorRequest = std::make_unique<MarineSensorDataRequestMsg>();
        //m_MsgBus.sendMessage(std::move(marineSensorRequest));
        CanMsg marineSensorRequest;
        marineSensorRequest.id = 710;
        marineSensorRequest.header.ide = 0;
        marineSensorRequest.header.length = 7;
        
        //Should be loop time here and boolean flag?

        for (auto& data : marineSensorRequest.data) {
            data = 0;
        }
        m_CANService.sendCANMessage(marineSensorRequest);
    }
}

void CANMarineSensorTransmissionNode::start() {
    runThread(CANMarineSensorTransmissionNode::CANSensorNodeThreadFunc);
}


/*
 * Should this be used at all??
 */
void CANMarineSensorTransmissionNode::CANSensorNodeThreadFunc(ActiveNode *nodePtr) {
    CANMarineSensorTransmissionNode* node = dynamic_cast<CANMarineSensorTransmissionNode*> (nodePtr);
    Timer timer;
    timer.start();
    while(true) {

        /*
         * Im assuming there should be a function here sending out data requests on a set interval.
         * Set interval is changed in database.
         * Also should have a flag to read values when hitting set checkpoint
         */


        timer.sleepUntil(node->m_LoopTime);
        timer.reset();
    }
}
