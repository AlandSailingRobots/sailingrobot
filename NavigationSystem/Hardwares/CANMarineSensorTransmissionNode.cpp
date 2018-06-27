

#include "CANMarineSensorTransmissionNode.h"

#include "../MessageBus/NodeIDs.h"
#include "../MessageBus/MessageTypes.h"
#include "../Messages/DataCollectionStartMsg.h"
#include "../SystemServices/Timer.h"
#include "CAN_Services/CanBusCommon/CanMessageHandler.h"
#include "CAN_Services/CanBusCommon/canbus_defs.h"

CANMarineSensorTransmissionNode::CANMarineSensorTransmissionNode(MessageBus &msgBus, CANService &canService)
                                                                : Node(NodeID::MarineSensorCANTransmission, msgBus),
                                                                 m_CANService(canService), m_arduinoSensorLoopTime(0),
                                                                 m_takeContinousSensorReadings(false) {
    msgBus.registerNode(*this, MessageType::DataRequest);
    msgBus.registerNode(*this, MessageType::DataCollectionStart);
    msgBus.registerNode(*this, MessageType::DataCollectionStop);
}

CANMarineSensorTransmissionNode::~CANMarineSensorTransmissionNode() {}


bool CANMarineSensorTransmissionNode::init() {
    return true;
}

void CANMarineSensorTransmissionNode::processMessage(const Message* message) {

    if(message->messageType() == MessageType::DataCollectionStop) {
        m_takeContinousSensorReadings = false;
    }
    else if(message->messageType() == MessageType::DataCollectionStart) {
        auto dataCollectionStartMsg = static_cast<const DataCollectionStartMsg*>(message);
        m_arduinoSensorLoopTime = dataCollectionStartMsg->getSensorReadingInterval();
        m_takeContinousSensorReadings = true;
    }

    /**
     * Last case is MessageType::DataRequest, but in all these cases a CAN Message should be sent to arduino
     */

    CanMessageHandler messageHandler(MSG_ID_MARINE_SENSOR_REQUEST);
    messageHandler.encodeMessage(REQUEST_CONTINOUS_READINGS_DATASIZE,m_takeContinousSensorReadings);
    messageHandler.encodeMessage(REQUEST_READING_TIME_DATASIZE,m_arduinoSensorLoopTime);

    CanMsg marineSensorRequest = messageHandler.getMessage();
    m_CANService.sendCANMessage(marineSensorRequest);
}
