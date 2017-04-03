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
