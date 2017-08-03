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
#include <map>

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
	LowLevelControllerNodeASPire,
	StateMessageListener,
	MessageVerifier,
	ActuatorNodeASPire,
	LowLevelControllerNodeJanet,
	DBLoggerNode,
	CANFeedbackReceiver,
	CourseRegulatorNode,
	SailControlNode,
	SpeedRegulatorNode,
	WingsailControlNode
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
		case NodeID::LowLevelControllerNodeASPire:
		return "LowLevelControllerNodeASPire";
		case NodeID::StateMessageListener:
		return "StateMessageListener";
		case NodeID::MessageVerifier:
		return "MessageVerifier";
		case NodeID::ActuatorNodeASPire:
		return "ActuatorNodeASPire";
		case NodeID::LowLevelControllerNodeJanet:
		return "LowLevelControllerJanet";
		case NodeID::DBLoggerNode:
		return "DBLoggerNode";
		case NodeID::CANFeedbackReceiver:
		return "CANFeedbackReceiver";
		case NodeID::CourseRegulatorNode:
		return "CourseRegulatorNode";
		case NodeID::SailControlNode:
		return "SailControlNode";
		case NodeID::SpeedRegulatorNode:
		return "SpeedRegulatorNode";
		case NodeID::WingsailControlNode:
		return "WingsailControlNode";
	}
	return "";
}

inline NodeID configToNode(std::string config){
	std::map <std::string, NodeID> nodeConfigs;
	nodeConfigs["config_arduino"]=			NodeID::Arduino;
	nodeConfigs["config_buffer"]=			NodeID::Compass;
	nodeConfigs["config_lidar"]=			NodeID::ColorDetection;
	nodeConfigs["config_courseRegulator"]=	NodeID::CourseRegulatorNode;
	nodeConfigs["config_dblogger"]=			NodeID::DBLoggerNode;
	nodeConfigs["config_GPSD"]=				NodeID::GPS;
	nodeConfigs["config_httpsync"]=			NodeID::HTTPSync;
	nodeConfigs["config_rudderActuator"]=	NodeID::RudderActuator;
	nodeConfigs["config_sailActuator"]=		NodeID::SailActuator;
	nodeConfigs["config_sailControl"]=		NodeID::SailControlNode;
	nodeConfigs["config_sailingLogic"]=		NodeID::SailingLogic;
	nodeConfigs["config_simulator"]=		NodeID::Simulator;
	nodeConfigs["config_speedRegulator"]=	NodeID::SpeedRegulatorNode;
	nodeConfigs["config_stateEstimation"]=	NodeID::StateEstimation;
	nodeConfigs["config_wingsailControl"]=	NodeID::WingsailControlNode;
	nodeConfigs["config_windState"]=		NodeID::WindSensor;
	//nodeConfigs["config_wind_vane"]=		NodeID::WindVane;
	nodeConfigs["config_xbee"]=				NodeID::xBeeSync;
	return nodeConfigs.find(config)->second;
}
