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
	if(PGN == PGN130306){
		uint8_t SID, Ref;
		float WS, WA;
		parsePGN130306(N2kMsg &Msg, SID, WS, WA, Ref);
		std::cout << "SID: " << (int)SID << " ";
		std::cout << "Windspeed: "<< WS << " ";
		std::cout << "Windangle: "<< WA << " ";
		std::cout << "Reference: "<< (int)Ref << std::endl;
	}
	else if()
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