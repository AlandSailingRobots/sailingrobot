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
                  int arduinoPressure,
                  int arduinoRudder,
                  int arduinoSheet,
                  int arduinoBattery,
                  float currentsensordataCurrent,
                  float currentsensordataVoltage,
                  SensedElement currentsensordataElement)
   	: Message(MessageType::PowerTrack, sourceID, destinationID),
   	  m_ArduinoPressure(arduinoPressure),
   	  m_ArduinoRudder(arduinoRudder),
   	  m_ArduinoSheet(arduinoSheet),
   	  m_ArduinoBattery(arduinoBattery),
   	  m_CurrentSensorDataCurrent(currentsensordataCurrent),
   	  m_CurrentSensorDataVoltage(currentsensordataVoltage),
   	  m_CurrentSensorDataElement(currentsensordataElement) {}

   	PowerTrackMsg(int arduinoPressure,
   		            int arduinoRudder,
   		            int arduinoSheet,
   		            int arduinoBattery,
   		            float currentsensordataCurrent,
   		            float currentsensordataVoltage,
   		            SensedElement currentsensordataElement)
   	: Message(MessageType::PowerTrack, NodeID::None, NodeID::None),
   	  m_ArduinoPressure(arduinoPressure),
   	  m_ArduinoRudder(arduinoRudder),
   	  m_ArduinoSheet(arduinoSheet),
   	  m_ArduinoBattery(arduinoBattery),
   	  m_CurrentSensorDataCurrent(currentsensordataCurrent),
   	  m_CurrentSensorDataVoltage(currentsensordataVoltage),
   	  m_CurrentSensorDataElement(currentsensordataElement) {}

   	PowerTrackMsg(MessageDeserialiser deserialiser) : Message(deserialiser) {
   		if (!deserialiser.readInt(m_ArduinoPressure) || !deserialiser.readInt(m_ArduinoRudder) ||
   			!deserialiser.readInt(m_ArduinoSheet) || !deserialiser.readInt(m_ArduinoBattery) ||
   			!deserialiser.readFloat(m_CurrentSensorDataCurrent) || 
   			!deserialiser.readFloat(m_CurrentSensorDataVoltage) ||
   			!deserialiser.readInt((int&)m_CurrentSensorDataElement)) {
	        m_valid = false;
        }
   	}

   	virtual ~PowerTrackMsg() {}

   	int arduinoPressure() { return m_ArduinoPressure; }
   	int arduinoRudder() { return m_ArduinoRudder; }
   	int arduinoSheet() { return m_ArduinoSheet; }
   	int arduinoBattery() { return m_ArduinoBattery; }
   	float currentsensordataCurrent() { return m_CurrentSensorDataCurrent; }
   	float currentsensordataVoltage() { return m_CurrentSensorDataVoltage; }
   	SensedElement currentsensordataElement() { return m_CurrentSensorDataElement; }


   	///---------------------------------------------------------------------------
    /// Serialises the message into a MessageSerialiser
    ///---------------------------------------------------------------------------
    virtual void Serialise(MessageSerialiser& serialiser) const {
    	Message::Serialise(serialiser);

    	serialiser.serialise(m_ArduinoPressure);
    	serialiser.serialise(m_ArduinoRudder);
    	serialiser.serialise(m_ArduinoSheet);
    	serialiser.serialise(m_ArduinoBattery);
    	serialiser.serialise(m_CurrentSensorDataCurrent);
    	serialiser.serialise(m_CurrentSensorDataVoltage);
    	serialiser.serialise((uint8_t)m_CurrentSensorDataElement);
    }

  private:
  	int m_ArduinoPressure;
  	int m_ArduinoRudder;
  	int m_ArduinoSheet;
  	int m_ArduinoBattery;
  	float m_CurrentSensorDataCurrent;
  	float m_CurrentSensorDataVoltage;
  	SensedElement m_CurrentSensorDataElement;
 };

#endif //NAVIGATIONSYSTEM_POWERTRACKMSG_H