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

#include "Message.h"


class ArduinoDataMsg : public Message {
public:
	ArduinoDataMsg(NodeID destinationID, NodeID sourceID, int pressure, int rudder, int sheet, int battery, int RC)
		:Message(MessageType::ArduinoData, sourceID, destinationID), m_pressure(pressure), m_rudder(rudder), m_sheet(sheet), m_battery(battery), m_RC(RC)
	{ }

	ArduinoDataMsg(int pressure, int rudder, int sheet, int battery, int RC)
		:Message(MessageType::ArduinoData, NodeID::None, NodeID::None), m_pressure(pressure), m_rudder(rudder), m_sheet(sheet), m_battery(battery), m_RC(RC)
	{ }

	ArduinoDataMsg(MessageDeserialiser deserialiser)
		:Message(deserialiser)
	{
		if(	!deserialiser.readInt(m_pressure) ||
			!deserialiser.readInt(m_rudder) ||
			!deserialiser.readInt(m_sheet) ||
			!deserialiser.readInt(m_battery) ||
			!deserialiser.readInt(m_RC))
		{
			m_valid = false;
		}
	}

	virtual ~ArduinoDataMsg() { }

	int pressure() { return m_pressure; }
	int rudder() { return m_rudder; }
	int sheet() { return m_sheet; }
    int battery() { return m_battery; }
	int RC() { return m_RC; }

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
		serialiser.serialise(m_RC);
	}

private:
	int m_pressure;
	int m_rudder;
	int m_sheet;
    int m_battery;
	int m_RC;
};
