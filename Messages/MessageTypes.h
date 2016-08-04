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


#include <string>


enum class MessageType {
	DataRequest = 0,
	WindData,
	CompassData,
	GPSData,
	ServerConfigsReceived,
	ServerWaypointsReceived,
	LocalConfigChange,
	LocalWaypointChange,
	ActuatorCommand,
	ActuatorFeedback,
	ArduinoData,
	VesselState,
	WaypointData,
	WindVaneCommand,
	CourseData
};

inline std::string msgToString(MessageType msgType)
{
	switch(msgType)
	{
	case MessageType::DataRequest:
		return "DataRequest";
	case MessageType::WindData:
		return "WindData";
	case MessageType::CompassData:
		return "CompassData";
	case MessageType::GPSData:
		return "GPSData";
	case MessageType::ServerConfigsReceived:
		return "ServerConfigReceived";
	case MessageType::ServerWaypointsReceived:
		return "ServerWaypointsReceived";
	case MessageType::LocalConfigChange:
		return "LocalConfigChange";
	case MessageType::LocalWaypointChange:
		return "LocalWaypointChange";
	case MessageType::ActuatorCommand:
		return "ActuatorCommand";
	case MessageType::ActuatorFeedback:
		return "ActuatorFeedback";
	case MessageType::ArduinoData:
		return "ArduinoData";
	case MessageType::VesselState:
		return "VesselState";
	case MessageType::WaypointData:
		return "WaypointData";
	case MessageType::WindVaneCommand:
		return "WindVaneCommand";
	case MessageType::CourseData:
		return "CourseData";
	}
	return "";
}
