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
	xBeeSync,
	SailingLogic,
	ColorDetection,
	CollisionAvoidanceBehaviour,
	Lidar,
	Simulator,
	LocalNavigationModule,
	LowLevelController,
	StateEstimation,
	MessagePrinter,
	WindStateNode,
	StateMessageListener
	MessageVerifier
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
		case NodeID::xBeeSync:
		return "xBeeSync";
		case NodeID::ColorDetection:
		return "ColorDetection";
		case NodeID::CollisionAvoidanceBehaviour:
		return "CollisionAvoidanceBehaviour";
		case NodeID::Lidar:
		return "Lidar";
		case NodeID::Simulator:
		return "Simulator";
		case NodeID::LocalNavigationModule:
		return "Local Navigation Module";
		case NodeID::LowLevelController:
		return "Low Level Controller";
		case NodeID::StateEstimation:
		return "StateEstimation";
		case NodeID::MessagePrinter:
		return "MessagePrinter";
		case NodeID::WindStateNode:
		return "WindStateNode";
		case NodeID::StateMessageListener:
		return "StateMessageListener";
		case NodeID::MessageVerifier:
		return "MessageVerifier";
	}
	return "";
}
