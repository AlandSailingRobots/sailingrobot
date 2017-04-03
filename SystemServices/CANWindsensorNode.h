/****************************************************************************************
 *
 * File:
 * 		CANWindsensorNode.h
 *
 * Purpose:
 *		Process messages from the CAN-Service
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/
#include "SystemService/CANPNGReceiver.h"


#pragma once

class CANWindsensorNode : public Node, public CANPNGReceiver
{
public:
	CANWindsensorNode(MessageBus& msgBus, NodeID id, float windDir, float windSpeed, float windTemperature);
	~CANWindsensorNode();

	/* data */
	virtual void processPGN(std::vector<uint8_t> Data, uint32_t PGN) = 0;


	///----------------------------------------------------------------------------------
 	/// Setups the actuator.
 	///
 	///----------------------------------------------------------------------------------
	virtual bool init();


	///----------------------------------------------------------------------------------
 	/// Processes CANWindsensorMsgs
 	///
 	///----------------------------------------------------------------------------------
	virtual void processMessage(const Message* message);

private:
	float m_WindDir;
	float m_WindSpeed;
	float m_WindTemperature;

};