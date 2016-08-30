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
	ActuatorPosition,
	ArduinoData,
	VesselState,
	WaypointData,
	CourseData,
    ObstacleVector,
    CollisionAvoidance,
    ObstaclePosition
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
	case MessageType::ActuatorPosition:
		return "ActuatorPosition";
	case MessageType::ArduinoData:
		return "ArduinoData";
	case MessageType::VesselState:
		return "VesselState";
	case MessageType::WaypointData:
		return "WaypointData";
	case MessageType::CourseData:
		return "CourseData";
    case MessageType::ObstacleVector:
        return "ObstacleVector";
    case MessageType::CollisionAvoidance:
        return "CollisionAvoidance";
    case MessageType::ObstaclePosition:
        return  "ObstaclePosition";
	}
	return "";
}
