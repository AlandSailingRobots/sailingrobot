/****************************************************************************************
*
* File:
* 		CurrentSensorDataMsg.h
*
* Purpose:
*		A CurrentSensorDataMsg contains information about the sensed current (and voltage)
*      of a certain element (whole unit or single actuator)
*
* Developer Notes:
*
*
***************************************************************************************/

#pragma once

#include "MessageBus/Message.h"

enum SensedElement : uint8_t { SAILDRIVE = 1, WINDVANE_SWITCH = 2, WINDVANE_ANGLE = 3, ACTUATOR_UNIT = 4 };

class CurrentSensorDataMsg : public Message {
public:
    CurrentSensorDataMsg(NodeID destinationID, NodeID sourceID, uint16_t current, uint16_t voltage, SensedElement element)
    :Message(MessageType::CurrentSensorData, sourceID, destinationID), m_current(current), m_voltage(voltage), m_element(element) { }

    CurrentSensorDataMsg(uint16_t current, uint16_t voltage, SensedElement element)
    :Message(MessageType::CurrentSensorData, NodeID::None, NodeID::None), m_current(current), m_voltage(voltage), m_element(element)
    {

    }

    CurrentSensorDataMsg(MessageDeserialiser deserialiser)
    :Message(deserialiser)
    {
        uint8_t element = 0;
        if(	!deserialiser.readUint16_t(m_current) ||
        !deserialiser.readUint16_t(m_voltage) ||
        !deserialiser.readUint8_t(element)
    )
    {
        m_valid = false;
    }
    m_element = (SensedElement) element;
}

virtual ~CurrentSensorDataMsg() { }

///----------------------------------------------------------------------------------
/// Serialises the message into a MessageSerialiser
///----------------------------------------------------------------------------------
virtual void Serialise(MessageSerialiser& serialiser) const
{
    Message::Serialise(serialiser);

    serialiser.serialise(m_current);
    serialiser.serialise(m_voltage);
    serialiser.serialise((uint8_t)m_element);
}

uint16_t getCurrent() const { return m_current; }
uint16_t getVoltage() const { return m_voltage; }
SensedElement getSensedElement() const { return m_element; }
std::string getSensedElementStr () const
{
    std::string elem;

    switch(m_element)
    {
        case SAILDRIVE:
        elem = "saildrive";
        break;

        case WINDVANE_SWITCH:
        elem = "windvane switch";
        break;

        case WINDVANE_ANGLE:
        elem = "windvane angle";
        break;

        case ACTUATOR_UNIT:
        elem = "actuator unit";
        break;

        default:
        elem = "undefined";
        break;
    }

    return elem;
}

private:
    uint16_t m_current;			// in mA
    uint16_t m_voltage;			// in mV
    SensedElement m_element;    // the element measured
};
