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

#pragma once

#include <mutex>
#include <vector>
#include <iostream>
#include <chrono>
#include <thread>

#include "Hardwares/CAN_Services/CANPGNReceiver.h"
#include "DataBase/DBHandler.h"
#include "Hardwares/CAN_Services/CANService.h"
#include "MessageBus/ActiveNode.h"
#include "Messages/WindDataMsg.h"
#include "SystemServices/Timer.h"


class CANWindsensorNode : public CANPGNReceiver, public ActiveNode {
public:
	CANWindsensorNode(MessageBus& msgBus, DBHandler& dbhandler, CANService& can_service, double loopTime);
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
	/// Update values from the database as the loop time of the thread and others parameters
	///----------------------------------------------------------------------------------
    void updateConfigsFromDB();

	///----------------------------------------------------------------------------------
 	/// Attempts to connect to the wind sensor.
 	///
 	///----------------------------------------------------------------------------------
	virtual bool init() {return true;};

	virtual void processMessage(const Message* message);

	void start();

private:

	const int DATA_OUT_OF_RANGE	=	-2000;

	static void CANWindSensorNodeThreadFunc(ActiveNode* nodePtr);

	float m_WindDir;			// NOTE : degree 0 - 360 (273)
	float m_WindSpeed;			// NOTE : m/s ou knots
	float m_WindTemperature;	// NOTE : in degree Celsius
	double m_LoopTime;			// in seconds (ex : 0.5 s)
	DBHandler& m_db;

	std::mutex m_lock;
	std::vector<uint32_t> PGNs {130306, 130311};
};
