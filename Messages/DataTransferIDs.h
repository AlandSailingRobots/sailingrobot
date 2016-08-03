/****************************************************************************************
 *
 * File:
 * 		CANID.h
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


enum class CANID {
	ActuatorCommandsOnly = 0x10,
	ActuatorAndWindVaneCommands = 0x11,
	ActuatorFeedbackOnly = 0x20,
	ActuatorAndWindFeedback = 0x21

};

enum class DataID {
	CANBus = 0

};

inline std::string CANIDToString(MessageType msgType)
{
	switch(msgType)
	{
	case CANID::ActuatorCommandsOnly:
		return "ActuatorCommandsOnly";
	case CANID::ActuatorAndWindVaneCommands:
		return "ActuatorAndWindVaneCommands";
	case CANID::ActuatorFeedbackOnly:
		return "ActuatorFeedbackOnly";
	case CANID::ActuatorAndWindFeedback:
		return "ActuatorAndWindFeedback";
	}
	return "";
}

inline int CANIDtoDataLength(MessageType msgType)
{
	switch(msgType)
	{
	case CANID::ActuatorCommandsOnly:
		return 4;
	case CANID::ActuatorAndWindVaneCommands:
		return 5;
	case CANID::ActuatorFeedbackOnly:
		return 4;
	case CANID::ActuatorAndWindFeedback:
		return 8;
	}
	return "";
}
