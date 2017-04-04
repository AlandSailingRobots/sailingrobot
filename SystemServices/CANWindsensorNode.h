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
	// This gave error when put in the .cpp-file, should be fixed?
	CANWindsensorNode(MessageBus& msgBus, CANService& can_service)
	 : CANPGNReceiver(can_service, PGNs), Node(NodeID::WindSensor, msgBus)
	 {
		 m_WindDir  = 0;
	     m_WindSpeed = 0;
	     m_WindTemperature = 0;
	 }


	~CANWindsensorNode();

	/* data */
	 void processPGN(N2kMsg &NMsg);


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
<<<<<<< HEAD
	virtual bool init() = 0;
	virtual void processMessage(const Message* message){};
=======
	bool init();
	void processMessage(const Message* message){};
>>>>>>> 038cac4d51f8cc66f9224669e70bcf009ab2ec93

	std::vector<uint32_t> PGNs {130306, 130311};


private:
	float m_WindDir;
	float m_WindSpeed;
	float m_WindTemperature;
};

<<<<<<< HEAD
=======

>>>>>>> 038cac4d51f8cc66f9224669e70bcf009ab2ec93
