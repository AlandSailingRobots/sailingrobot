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
	ArduinoDataMsg(NodeID destinationID, NodeID souRadio_ControllereID, int pressure, int rudder, int sheet, int battery, int Radio_Controller)
		:Message(MessageType::ArduinoData, souRadio_ControllereID, destinationID), m_pressure(pressure), m_rudder(rudder), m_sheet(sheet), m_battery(battery), m_Radio_Controller(Radio_Controller)
	{ }

	ArduinoDataMsg(int pressure, int rudder, int sheet, int battery, int Radio_Controller)
		:Message(MessageType::ArduinoData, NodeID::None, NodeID::None), m_pressure(pressure), m_rudder(rudder), m_sheet(sheet), m_battery(battery), m_Radio_Controller(Radio_Controller)
	{ }

	ArduinoDataMsg(MessageDeserialiser deserialiser)
		:Message(deserialiser)
	{
		if(	!deserialiser.readInt(m_pressure) ||
			!deserialiser.readInt(m_rudder) ||
			!deserialiser.readInt(m_sheet) ||
			!deserialiser.readInt(m_battery) ||
			!deserialiser.readInt(m_Radio_Controller))
		{
			m_valid = false;
		}
	}

	virtual ~ArduinoDataMsg() { }

	int pressure() { return m_pressure; }
	int rudder() { return m_rudder; }
	int sheet() { return m_sheet; }
    int battery() { return m_battery; }
	int Radio_Controller() const  { return m_Radio_Controller; }

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
		serialiser.serialise(m_Radio_Controller);
	}

private:
	int m_pressure;
	int m_rudder;
	int m_sheet;
    int m_battery;
	int m_Radio_Controller;
};
