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

enum SensedElement : uint8_t {
    SOLAR_PANEL   = 0,
    POWER_UNIT    = 1,
    ACTUATOR_UNIT = 2,
};

class CurrentSensorDataMsg : public Message {
   public:
    CurrentSensorDataMsg(NodeID destinationID,
                         NodeID sourceID,
                         float current,
                         float voltage,
                         SensedElement element)
        : Message(MessageType::CurrentSensorData, sourceID, destinationID),
          m_current(current),
          m_voltage(voltage),
          m_element(element) {}

    CurrentSensorDataMsg(float current, float voltage, SensedElement element)
        : Message(MessageType::CurrentSensorData, NodeID::None, NodeID::None),
          m_current(current),
          m_voltage(voltage),
          m_element(element) {}

    CurrentSensorDataMsg(MessageDeserialiser deserialiser) : Message(deserialiser) {
        uint8_t element = 0;
        if (!deserialiser.readFloat(m_current) || !deserialiser.readFloat(m_voltage) ||
            !deserialiser.readUint8_t(element)) {
            m_valid = false;
        }
        m_element = (SensedElement)element;
    }

    virtual ~CurrentSensorDataMsg() {}

    ///----------------------------------------------------------------------------------
    /// Serialises the message into a MessageSerialiser
    ///----------------------------------------------------------------------------------
    virtual void Serialise(MessageSerialiser& serialiser) const {
        Message::Serialise(serialiser);

        serialiser.serialise(m_current);
        serialiser.serialise(m_voltage);
        serialiser.serialise((uint8_t)m_element);
    }

    float getCurrent() const { return m_current; }
    float getVoltage() const { return m_voltage; }
    SensedElement getSensedElement() const { return m_element; }
    std::string getSensedElementStr() const {
        std::string elem;

        switch (m_element) {  // added "' '" to have proper sql string when using sal command lines
                              // in dbloggernode
            case SOLAR_PANEL:
                elem = "Solar panel";
                break;

            case POWER_UNIT:
                elem = "Power unit";
                break;

            case ACTUATOR_UNIT:
                elem = "Actuator unit";
                break;

            default:
                elem = "undefined";
                break;
        }

        return elem;
    }

   private:
    float m_current;          // in mA
    float m_voltage;          // in mV
    SensedElement m_element;  // the element measured
};
