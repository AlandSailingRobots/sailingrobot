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

 CANWindsensorNode::CANWindsensorNode(MessageBus& msgBus, NodeID id, float windDir, float windSpeed, float windTemperature)
 	:Node(id, msgBus), m_WindDir(windDir), m_WindSpeed(m_WindSpeed), m_WindTemperature(windTemperature)
 {
 		msgBus.registerNode(*this, MessageType::WindDataMsg);
 }

 bool CANWindsensorNode::init()
{
  m_fd = serialOpen(m_PortName.c_str(),m_BaudRate);


	if(m_fd!=-1)
	{
		m_Initialised = true;
	}
	else
	{
		Logger::error("%s Cannot open %s as windsensor: %s", __PRETTY_FUNCTION__,m_PortName.c_str(),strerror(errno));
	}

	return m_Initialised;
}

void CANWindsensorNode::start()
{
	if(m_Initialised)
	{
		runThread(WindSensorThread);
	}
	else
	{
		Logger::error("%s Cannot start wind sensor thread as the node was not correctly initialised!", __PRETTY_FUNCTION__);
	}
}

void CANWindsensorNode::processPGN(N2kMsg &NMsg, uint32_t PGN)
{
	//TODO:: parse message 
	if(NMsg.PGN == 130306){
		uint8_t SID, Ref;
		float WS, WA;
		parsePGN130306(N2kMsg &Msg, SID, WS, WA, Ref);
		std::cout << "SID: " << (int)SID << " ";
		std::cout << "Windspeed: "<< WS << " ";
		std::cout << "Windangle: "<< WA << " ";
		std::cout << "Reference: "<< (int)Ref << std::endl;
	}
	else if(NMsg.PGN == 130311){
		uint8_t SID, TI, HI;
		float Temp, Hum, AP;
		ParsePGN130311(NMsg, SID, TI, HI, Temp, Hum, AP);
		std::cout << "SID: " << (int)SID << " ";
		std::cout << "TemperatureInstance: " << (int)TI << " ";
		std::cout << "HumidityInstance: " << (int)HI << " ";
		std::cout << "Temperature: " << Temp << " ";
		std::cout << "Humidity: " << Hum << " ";
		std::cout << "AtmosphericPressure: " << AP << std::endl;
	}
	else if(NMsg.PGN == 130312){
		uint8_t SID, TI, TS;
		float ATemp, STemp;
		ParsePGN130312(NMsg, SID, TI, TS, ATemp, STemp);
		std::cout << "SID: " << (int)SID << " ";
		std::cout << "TemperatureInstance: " << (int)TI << " ";
		std::cout << "TemperatureSource: " << (int)TS << " ";
		std::cout << "Actual Temperature: " << ATemp << " ";
		std::cout << "Set Temperature: " << STemp << std::endl;
	}
}

void CANWindsensorNode::parsePGN130306(N2kMsg &NMsg, uuint8_t &SID, float &WindSpeed,				//WindData
	float &WindAngle, uint8_t &Reference)
{
	SID = NMsg.Data[0];
	uint16_t tmp = NMsg.Data[1] | (NMsg.Data[2]<<8);
	WindSpeed = tmp*0.01;
	tmp = NMsg.Data[3] | (NMsg.Data[4]<<8);
	WindAngle = tmp*0.0001;
	Reference = NMsg.Data[5] & 0x07;
}

void CANWindsensorNode::parsePGN130311(N2kMsg &Msg, uint8_t &SID, uint8_t &TemperatureInstance,	//Environmental Parameters
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

void CANWindsensorNode::ParsePGN130312(N2kMsg &NMsg, uint8_t &SID, uint8_t &TemperatureInstance,	//Temperature
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