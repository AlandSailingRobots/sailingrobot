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

void CANWindsensorNode::processPGN(std::vector<uint8_t> Data, uint32_t PGN)
{
	//TODO:: parse message and send to message bus
	if(PGN == PGN130306){
		uint8_t SID, Ref;
		float WS, WA;
		parsePGN130306(Data, SID, WS, WA, Ref);
		std::cout << "SID: " << (int)SID << " ";
		std::cout << "Windspeed: "<< WS << " ";
		std::cout << "Windangle: "<< WA << " ";
		std::cout << "Reference: "<< (int)Ref << std::endl;
	}
}

void CANWindsensorNode::parsePGN130306(std::vector<uint8_t> Data, uuint8_t &SID, float &WindSpeed,				//WindData
	float &WindAngle, uint8_t &Reference)
{
	SID = Data[0];
	uint16_t tmp = Data[1] | (Data[2]<<8);
	WindSpeed = tmp*0.01;
	tmp = Data[3] | (Data[4]<<8);
	WindAngle = tmp*0.0001;
	Reference = Data[5] & 0x07;
}