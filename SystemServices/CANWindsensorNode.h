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
#include "CANPGNReceiver.h"
#include "Nodes/Node.h"
#include "CANService.h"

#include <vector>

#pragma once


class CANWindsensorNode : public CANPGNReceiver, public Node
{
public:
	CANWindsensorNode(MessageBus& msgBus, CANService& can_service, float windDir, float windSpeed, float windTemperature);

	~CANWindsensorNode();

	/* data */
	virtual void processPGN(N2kMsg &NMsg) = 0;


    void parsePGN130306(N2kMsg &NMsg, uint8_t &SID, float &WindSpeed,				//WindData
					float &WindAngle, uint8_t &Reference);

    void parsePGN130311(N2kMsg &Msg, uint8_t &SID, uint8_t &TemperatureInstance,	//Environmental Parameters
					uint8_t &HumidityInstance, float &Temperature,
					float &Humidity, float &AtmosphericPressure);

    void parsePGN130312(N2kMsg &NMsg, uint8_t &SID, uint8_t &TemperatureInstance,	//Temperature
					uint8_t &TemperatureSource, float &ActualTemperature,
					float &SetTemperature);

    void parsePGN130314(N2kMsg &Msg, uint8_t &SID, uint8_t &PressureInstance,		//ActualPressure
					uint8_t &PressureSource, double &Pressure);


	///----------------------------------------------------------------------------------
 	/// Attempts to connect to the wind sensor.
 	///
 	///----------------------------------------------------------------------------------
	virtual bool init();
	std::vector<uint32_t> PGNs {130306, 130311};


private:
	float m_WindDir;
	float m_WindSpeed;
	float m_WindTemperature;
};

