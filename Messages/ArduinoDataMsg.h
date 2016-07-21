/****************************************************************************************
 *
 * File:
 * 		ArduinoDataMsg.h
 *
 * Purpose:
 *		A ArduinoDataMsg contains arduino data such as pressure, rudder, sheet, battery.
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/

#pragma once

#include "Message.h"


class ArduinoDataMsg : public Message {
public:
	ArduinoDataMsg(NodeID destinationID, NodeID sourceID, int pressure, int rudder, int sheet, int battery)
		:Message(MessageType::ArduinoData, sourceID, destinationID), m_pressure(pressure), m_rudder(rudder), m_sheet(sheet), m_battery(battery)
	{ }

	ArduinoDataMsg(int pressure, int rudder, int sheet, int battery)
		:Message(MessageType::ArduinoData, NodeID::None, NodeID::None), m_pressure(pressure), m_rudder(rudder), m_sheet(sheet), m_battery(battery)
	{ }

	virtual ~ArduinoDataMsg() { }

	int pressure() { return m_pressure; }
	int rudder() { return m_rudder; }
	int sheet() { return m_sheet; }
    int battery() { return m_battery; }

private:
	int m_pressure;
	int m_rudder;
	int m_sheet;
    int m_battery;
};
