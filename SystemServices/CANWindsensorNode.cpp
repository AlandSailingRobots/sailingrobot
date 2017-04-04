/****************************************************************************************
 *
 * File:
 * 		CANWindsensorNode.cpp
 *
 * Purpose:
 *		 Process messages from the CAN-Service.
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/

#include "CANWindsensorNode.h"
#include "Messages/WindDataMsg.h"


/* CANWindsensorNode::CANWindsensorNode(MessageBus& msgBus, CANService& can_service)
 : CANPGNReceiver(can_service, PGNs), Node(NodeID::WindSensor, msgBus)
 {
 	m_WindDir  = 0;
    m_WindSpeed = 0;
    m_WindTemperature = 0;
 } */

 bool CANWindsensorNode::init()
{
 	return true;
}

void CANWindsensorNode::processPGN(N2kMsg &NMsg)
{
	if(NMsg.PGN == 130306){
		uint8_t SID, Ref;
		float WS, WA;
		parsePGN130306(NMsg, SID, WS, WA, Ref);

		m_WindDir = WA;
		m_WindSpeed = WS;

		MessagePtr windData = std::make_unique<WindDataMsg>(m_WindDir, m_WindTemperature, m_WindSpeed);
		m_MsgBus.sendMessage(std::move(windData));

	}
	else if(NMsg.PGN == 130311)
	{
		uint8_t SID, TI, HI;
		float Temp, Hum, AP;
		parsePGN130311(NMsg, SID, TI, HI, Temp, Hum, AP);

		m_WindTemperature = Temp;

		MessagePtr windData = std::make_unique<WindDataMsg>(m_WindDir, m_WindTemperature, m_WindSpeed);
		m_MsgBus.sendMessage(std::move(windData));
	}
	else if(NMsg.PGN == 130312)
	{
		uint8_t SID, TI, TS;
		float ATemp, STemp;
		parsePGN130312(NMsg, SID, TI, TS, ATemp, STemp);
		parsePGN130312(NMsg, SID, TI, TS, ATemp, STemp);
	}
	else if (NMsg.PGN == 130314)
	{
		uint8_t SID, PI, PS;
		double P;
		parsePGN130314(NMsg, SID, PI, PS, P);
		parsePGN130314(NMsg, SID, PI, PS, P);
	}
}

void CANWindsensorNode::parsePGN130306(N2kMsg &NMsg, uint8_t &SID, float &WindSpeed,				//WindData
	float &WindAngle, uint8_t &Reference)
{
	SID = NMsg.Data[0];
	uint16_t tmp = NMsg.Data[1] | (NMsg.Data[2]<<8);
	WindSpeed = tmp*0.01;
	tmp = NMsg.Data[3] | (NMsg.Data[4]<<8);
	WindAngle = tmp*0.0001;
	Reference = NMsg.Data[5] & 0x07;
}

void CANWindsensorNode::parsePGN130311(N2kMsg &NMsg, uint8_t &SID, uint8_t &TemperatureInstance,	//Environmental Parameters
	uint8_t &HumidityInstance, float &Temperature,
	float &Humidity, float &AtmosphericPressure)
{
	SID = NMsg.Data[0];
	TemperatureInstance = NMsg.Data[1] & 0x3f;
	HumidityInstance = NMsg.Data[1] >> 6;
	uint16_t tmp = NMsg.Data[2] | (NMsg.Data[3]<<8);
	Temperature = tmp*0.01;
	//tmp = NMsg.Data[4] | (NMsg.Data[5]<<8);
	//Humidity = tmp*0.004;
	Humidity = 0;
	tmp = NMsg.Data[6] | (NMsg.Data[7]<<8);
	AtmosphericPressure = tmp;		//hPa
}

void CANWindsensorNode::parsePGN130312(N2kMsg &NMsg, uint8_t &SID, uint8_t &TemperatureInstance,	//Temperature
					uint8_t &TemperatureSource, float &ActualTemperature,
					float &SetTemperature)
{
	SID = NMsg.Data[0];
	TemperatureInstance = NMsg.Data[1];
	TemperatureSource = NMsg.Data[2];
	uint16_t tmp = NMsg.Data[3] | (NMsg.Data[4]<<8);
	ActualTemperature = tmp*0.01;
	tmp = NMsg.Data[5] | (NMsg.Data[6]<<8);
	SetTemperature = tmp*0.01;
}

void CANWindsensorNode::parsePGN130314(N2kMsg &NMsg, uint8_t &SID, uint8_t &PressureInstance,		//ActualPressure
					uint8_t &PressureSource, double &Pressure)
{
	SID = NMsg.Data[0];
	PressureInstance = NMsg.Data[1];
	PressureSource = NMsg.Data[2];

	uint32_t tmp = NMsg.Data[3] | (NMsg.Data[4]<<8) | (NMsg.Data[5]<<16) | (NMsg.Data[6]<<24);
	Pressure = tmp / 1000.0f; 			//hPa
}
