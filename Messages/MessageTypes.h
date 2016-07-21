/****************************************************************************************
 *
 * File:
 * 		MessageTypes.h
 *
 * Purpose:
 *		Provides a enum containing all the message types. Used in the base message class
 *		so that when a message pointer is passed around you know what type of message to
 *		cast it to.
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/

#pragma once


enum class MessageType {
	DataRequest = 0,
	WindData,
	CompassData,
	GPSData,
	ArduinoData,
	VesselState,
	WaypointData
};
