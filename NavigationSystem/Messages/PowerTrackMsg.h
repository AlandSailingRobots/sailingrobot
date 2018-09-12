/****************************************************************************************
 *
 * File:
 * 		PowerTrackMsg.h
 *
 * Purpose:
 *		A PowerTrackMsg contains the power state of the vessel at a given time.
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/

#ifndef NAVIGATIONSYSTEM_POWERTRACKMSG_H
#define NAVIGATIONSYSTEM_POWERTRACKMSG_H

#include "../MessageBus/Message.h"

 class PowerTrackMsg : public Message {
   public:
   	PowerTrackMsg(NodeID destinationID,
                  NodeID sourceID,
                  float balance,
                  float currentsensordataCurrent,
                  float currentsensordataVoltage,
                  SensedElement currentsensordataElement)
   	: Message(MessageType::PowerTrack, sourceID, destinationID),
      m_PowerBalance(balance),
      m_CurrentSensorDataCurrent(currentsensordataCurrent),
      m_CurrentSensorDataVoltage(currentsensordataVoltage),
      m_CurrentSensorDataElement(currentsensordataElement) {}

   	PowerTrackMsg(float balance,
                  float currentsensordataCurrent,
                  float currentsensordataVoltage,
                  SensedElement currentsensordataElement)
   	: Message(MessageType::PowerTrack, NodeID::None, NodeID::None),
      m_PowerBalance(balance),
      m_CurrentSensorDataCurrent(currentsensordataCurrent),
      m_CurrentSensorDataVoltage(currentsensordataVoltage),
      m_CurrentSensorDataElement(currentsensordataElement) {}

   	PowerTrackMsg(MessageDeserialiser deserialiser) : Message(deserialiser) {
       if (!deserialiser.readFloat(m_PowerBalance) || !deserialiser.readFloat(m_CurrentSensorDataCurrent) ||
           !deserialiser.readFloat(m_CurrentSensorDataVoltage) ||
           !deserialiser.readInt((int&)m_CurrentSensorDataElement)) {
	        m_valid = false;
        }
   	}

    virtual void Serialise(MessageSerialiser& serialiser) const {
      Message::Serialise(serialiser);
      serialiser.serialise(m_PowerBalance); 
      serialiser.serialise(m_CurrentSensorDataCurrent);
      serialiser.serialise(m_CurrentSensorDataVoltage);  
      serialiser.serialise((uint8_t)m_CurrentSensorDataElement);
      }

   	virtual ~PowerTrackMsg() {}

    float getBalance() const { return m_PowerBalance; }
    float getCurrent() const { return m_CurrentSensorDataCurrent; }
    float getVoltage() const { return m_CurrentSensorDataVoltage; }
    SensedElement getElement() const { return m_CurrentSensorDataElement; }

   	///---------------------------------------------------------------------------
    /// Serialises the message into a MessageSerialiser
    ///---------------------------------------------------------------------------

  private:
    float m_PowerBalance;
    float m_CurrentSensorDataCurrent;
    float m_CurrentSensorDataVoltage;
    SensedElement m_CurrentSensorDataElement;
 };

#endif //NAVIGATIONSYSTEM_POWERTRACKMSG_H
