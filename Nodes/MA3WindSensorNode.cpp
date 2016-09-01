/****************************************************************************************
 *
 * File:
 * 		MA3WindSensorNode.cpp
 *
 * Purpose:
 *		
 *
 * Developer Notes:
 *
 ***************************************************************************************/

#include "MA3WindSensorNode.h"
#include "Messages/WindDataMsg.h"
#include "SystemServices/Logger.h"
#include "SystemServices/MaestroController.h"
#include "utility/SysClock.h"


#define MAX_VALUE		1023


MA3WindSensorNode::MA3WindSensorNode(MessageBus& msgBus, int channel)
	:ActiveNode(NodeID::WindSensor, msgBus), m_channel(channel)
{

}

MA3WindSensorNode::~MA3WindSensorNode()
{
	m_running = false;
}

bool MA3WindSensorNode::init()
{
	//return (readWindDirection() > -1);
	return true;
}

void MA3WindSensorNode::start()
{
	runThread(ma3ThreadFunc);
}

int MA3WindSensorNode::readWindDirection()
{
	int response = MaestroController::readCommand(MaestroCommands::GetPosition, m_channel);

	if(response == -1)
	{
		Logger::error("%s:%d Failed to read wind direction", __FILE__, __LINE__);
	}

	return response;
}

void MA3WindSensorNode::ma3ThreadFunc(void* nodePtr)
{
	MA3WindSensorNode* node = (MA3WindSensorNode*)nodePtr;
	node->m_running = true;

	while(node->m_running)
	{
		SysClock::sleepMS(500);
		int windDir = node->readWindDirection();

		if(windDir != -1)
		{
			windDir = (((windDir*1000) / MAX_VALUE) * 360) / 1000;
			//Logger::info("Wind Data: %d", windDir);
			MessagePtr windData = std::make_unique<WindDataMsg>(windDir, 0, 0);
			node->m_MsgBus.sendMessage(std::move(windData));
		}
		else
		{
			Logger::info("Failed to read wind");
		}
	}
}


