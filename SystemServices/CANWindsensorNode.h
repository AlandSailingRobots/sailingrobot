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

    void parsePGN130306(std::vector<uint8_t> Data, uuint8_t &SID, float &WindSpeed,				//WindData
					float &WindAngle, uint8_t &Reference);

	///----------------------------------------------------------------------------------
 	/// Setups the actuator.
 	///
 	///----------------------------------------------------------------------------------
	virtual bool init();


	///----------------------------------------------------------------------------------
 	/// Attempts to connect to the CV7 wind sensor.
 	///
 	///----------------------------------------------------------------------------------
	bool init();

	///----------------------------------------------------------------------------------
 	/// Starts the wind sensors thread so that it actively pumps data into the message
 	/// bus.
 	///
 	///----------------------------------------------------------------------------------
	void start();

private:
	bool m_Initialised;		// Indicates that the node was correctly initialised
	float m_WindDir;
	float m_WindSpeed;
	float m_WindTemperature;

	//Windsensor
	const uint32_t PGN130306 = 130306; 
	const uint32_t PGN130311 = 130311;
	const uint32_t PGN130312 = 130312;
	const uint32_t PGN130314 = 130314;

};