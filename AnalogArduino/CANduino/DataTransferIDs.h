/****************************************************************************************
 *
 * File:
 * 		DataID.h
 *
 * Purpose:
 *		Provides a enum containing all the data ID types.
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/

#pragma once

enum DataID {
	ActuatorCommands = 0x01,
	ActuatorAndWindVaneCommands = 0x02,
	ActuatorFeedback = 0x80,
	WindSensorFeedback = 0x40,
	AnalogFeedback = 0x20
};

int DataIDToByteLength(uint8_t dataID)
{
	switch(dataID)
	{
	case DataID::ActuatorCommands:
		return 4;
	case DataID::ActuatorAndWindVaneCommands:
		return 5;
	case DataID::ActuatorFeedback:
		return 4;
	case DataID::WindSensorFeedback:
		return 4;
	case DataID::AnalogFeedback:
		return 4;
	}
	return 0;
}
