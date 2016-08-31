/****************************************************************************************
 *
 * File:
 * 		MaestroController.h
 *
 * Purpose:
 *		Provides an interface to the USB MaestroController that controls the vessel's
 *		motors. This class is thread safe.
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/

#pragma once

#include <string>
#include <stdint.h>
#include <mutex>


enum class MaestroCommands {
	SetPosition = 0x84,
	SetSpeed = 0x87,
	SetAcceleration = 0x89,
	GetPosition = 0x90,
	GetMovingState = 0x93,
	GetError = 0xA1,
	SetHomePosition = 0xA2
};


class MaestroController {
public:
	///----------------------------------------------------------------------------------
	/// Initialises the Maestro controller, returns false if the maestro controller fails
	/// to initialise.
	///----------------------------------------------------------------------------------
	static bool init(std::string portName);

	///----------------------------------------------------------------------------------
	/// Writes a command to the maestro controller. Returns false if the command fails to
	/// work.
	///
	/// @param command	The command to issue.
	/// @param channel	The channel to issue the command on, default -1
	/// @param value	A argument value to write alongside the command, default -1
	///----------------------------------------------------------------------------------
	static bool writeCommand(MaestroCommands command, int channel = -1, int value = -1);

	///----------------------------------------------------------------------------------
	/// Reads a response from a command.
	///----------------------------------------------------------------------------------
	static int readResponse();

	static int readCommand(MaestroCommands command, int channel = -1);

	///----------------------------------------------------------------------------------
	/// Returns the last reported error from the Maestro hardware.
	///----------------------------------------------------------------------------------
	static int getError();
private:
	static std::mutex 	m_Mutex;
	static int 			m_Handle;
};
