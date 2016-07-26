/****************************************************************************************
 *
 * File:
 * 		NodeIDs.h
 *
 * Purpose:
 *		Contains all the Node IDs. A NodeID is a logically name ot describe what a node
 *		does. ITs primary purpose is to allow nodes to send messages to one another 
 *		directly instead of via the subscription system. It is also used for message 
 *		tracking purposes.
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/

#pragma once


#include <string>
 

enum class NodeID {
	None = 0,
	MessageLogger,
	WindSensor,
	Compass,
	GPS,
	HTTPSync,
	SailActuator,
	RudderActuator,
	Arduino,
	VesselState,
	Waypoint,
	SailingLogic
};

inline std::string nodeToString(NodeID id)
{
	switch(id)
	{
	case NodeID::None:
		return "None";
	case NodeID::MessageLogger:
		return "MessageLogger";
	case NodeID::WindSensor:
		return "WindSensor";
	case NodeID::Compass:
		return "Compass";
	case NodeID::GPS:
		return "GPS";
	case NodeID::HTTPSync:
		return "HTTPSync";
	case NodeID::SailActuator:
		return "SailActuator";
	case NodeID::RudderActuator:
		return "RudderActuator";
	case NodeID::Arduino:
		return "Arduino";
	case NodeID::VesselState:
		return "VesselState";
	case NodeID::Waypoint:
		return "Waypoint";
	case NodeID::SailingLogic:
		return "SailingLogic";
	}
	return "";
}
