/****************************************************************************************
 *
 * File:
 * 		ArduinoDataMsg.h
 *
 * Purpose:
 *		An ArduinoDataMsg contains arduino data such as pressure, rudder, sheet, battery.
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/

#pragma once

#include "MessageBus/Message.h"


class ArduinoDataMsg : public Message {
public:
	ArduinoDataMsg(NodeID destinationID, NodeID sourceID, int pressure, int rudder, int sheet, int battery)
		:Message(MessageType::ArduinoData, sourceID, destinationID), m_pressure(pressure), m_rudder(rudder), m_sheet(sheet), m_battery(battery)
	{ }

	ArduinoDataMsg(int pressure, int rudder, int sheet, int battery)
		:Message(MessageType::ArduinoData, NodeID::None, NodeID::None), m_pressure(pressure), m_rudder(rudder), m_sheet(sheet), m_battery(battery)
	{ }

	ArduinoDataMsg(MessageDeserialiser deserialiser)
		:Message(deserialiser)
	{
		if(	!deserialiser.readInt(m_pressure) ||
			!deserialiser.readInt(m_rudder) ||
			!deserialiser.readInt(m_sheet) ||
			!deserialiser.readInt(m_battery))
			
		{
			m_valid = false;
		}
	}

	virtual ~ArduinoDataMsg() { }

	int pressure() { return m_pressure; }
	int rudder() { return m_rudder; }
	int sheet() { return m_sheet; }
    int battery() { return m_battery; }
	

    ///----------------------------------------------------------------------------------
	/// Serialises the message into a MessageSerialiser
	///----------------------------------------------------------------------------------
	virtual void Serialise(MessageSerialiser& serialiser) const
	{
		Message::Serialise(serialiser);

		serialiser.serialise(m_pressure);
		serialiser.serialise(m_rudder);
		serialiser.serialise(m_sheet);
		serialiser.serialise(m_battery);
		
	}

private:
	int m_pressure;
	int m_rudder;
	int m_sheet;
  int m_battery;
	
};
