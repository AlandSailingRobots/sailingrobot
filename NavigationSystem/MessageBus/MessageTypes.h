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
	ObstacleVector,
	LidarData,
	CourseData,
	ExternalControl,
	RequestCourse,
	StateMessage,
	DesiredCourse,
	WindState,
	NavigationControl,
	ActuatorControlASPire,
	ASPireActuatorFeedback,
	SolarData,
	AISData
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
		case MessageType::ObstacleVector:
		return "ObstacleVector";
		case MessageType::LidarData:
		return "LidarData";
		case MessageType::CourseData:
		return "CourseData";
		case MessageType::ExternalControl:
		return "ExternalControl";
		case MessageType::RequestCourse:
		return "RequestCourse";
		case MessageType::StateMessage:
		return "StateMessage";
		case MessageType::DesiredCourse:
		return "DesiredCourse";
		case MessageType::WindState:
		return "WindState";
		case MessageType::NavigationControl:
		return "NavigationControl";
		case MessageType::ActuatorControlASPire:
		return "ActuatorControlASPire";
		case MessageType::ASPireActuatorFeedback:
		return "ASPireActuatorFeedback";
		case MessageType::SolarData:
		return "SolarData"
		case MessageType::AISData:
		return "AISData"
	}
	return "";
}
