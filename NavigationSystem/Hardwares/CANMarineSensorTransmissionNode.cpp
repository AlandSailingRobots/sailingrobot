

#include "CANMarineSensorTransmissionNode.h"

#include "MessageBus/NodeIDs.h"
#include "MessageBus/MessageTypes.h"
#include "Messages/DataCollectionStartMsg.h"
#include "SystemServices/Timer.h"


CANMarineSensorTransmissionNode::CANMarineSensorTransmissionNode(MessageBus &msgBus, CANService &canService)
                                                                : Node(NodeID::MarineSensorCANTransmission, msgBus),
                                                                 m_CANService(canService), m_arduinoSensorLoopTime(0),
                                                                 m_takeContinousSensorReadings(false) {
    msgBus.registerNode(*this, MessageType::DataRequest);
    msgBus.registerNode(*this, MessageType::DataCollectionStart);
    msgBus.registerNode(*this, MessageType::DataCollectionStop);
}

CANMarineSensorTransmissionNode::~CANMarineSensorTransmissionNode() {}

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
    CanMsg marineSensorRequest;
    fillCanMsg(marineSensorRequest);
    m_CANService.sendCANMessage(marineSensorRequest);
}

void CANMarineSensorTransmissionNode::fillCanMsg(CanMsg &message) {
    message.id = 710;
    message.header.ide = 0;
    message.header.length = 7;

    for (auto& data : message.data) {
        data = 0;
    }
    message.data[0] = static_cast<uint8_t>(m_takeContinousSensorReadings);
    message.data[1] = static_cast<uint8_t >(m_arduinoSensorLoopTime & 0xff);
    message.data[2] = static_cast<uint8_t >(m_arduinoSensorLoopTime >> 8);
    message.data[3] = static_cast<uint8_t >(m_arduinoSensorLoopTime >> 16);
    message.data[4] = static_cast<uint8_t >(m_arduinoSensorLoopTime >> 24);
}

