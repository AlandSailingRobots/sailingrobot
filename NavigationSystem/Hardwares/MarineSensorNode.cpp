
#include "MarineSensorNode.h"
#include "Messages/MarineSensorDataMsg.h"
#include "SystemServices/Logger.h"
#include "wiringPi.h"
#include "Math/Utility.h"
#include "SystemServices/Timer.h"
#include "AtlasScientificController/AtlasScientific.h"


// For std::this_thread
#include <chrono>
#include <thread>
#include <stdlib.h>

#define I2C_ADDRESS_TEMP			102
#define I2C_ADDRESS_COND			100
#define I2C_ADDRESS_PH				99

#define DATA_OUT_OF_RANGE 		-2000


MarineSensorNode::MarineSensorNode(MessageBus& msgBus, int miniWaitTime)
: Node(NodeID::MarineSensor, msgBus), m_miniWaitTime(miniWaitTime), m_Initialised(false)

{
	m_ph = DATA_OUT_OF_RANGE;
	m_conductivety = DATA_OUT_OF_RANGE;
	m_temp = DATA_OUT_OF_RANGE;
	
	msgBus.registerNode(*this, MessageType::DataRequest);
	
}

MarineSensorNode::~MarineSensorNode()
{
	
}



bool MarineSensorNode::init()
{
	m_Initialised = true;

	if(m_I2C_TEMP.init(I2C_ADDRESS_TEMP))
	{
		m_I2C_TEMP.beginTransmission();
		std::string response = m_I2C_TEMP.ASQUERY("I");
		printf("received %s request %02x\n", response.c_str(), I2C_ADDRESS_TEMP);
		m_I2C_TEMP.endTransmission();
		if(response.empty())
		{
			m_Initialised = false;
			Logger::error("%s Failed to successfully read from the marine temperature sensor, check connection!", __PRETTY_FUNCTION__);
		}
	}
	else
	{
		Logger::error("%s Failed to obtain Temperature I2C file descriptor", __PRETTY_FUNCTION__);
		m_Initialised = false;
	}
	if(m_I2C_COND.init(I2C_ADDRESS_COND))
	{
		m_I2C_COND.beginTransmission();
		std::string response = m_I2C_COND.ASQUERY("I");
		printf("received %s request %02x\n", response.c_str(), I2C_ADDRESS_COND);
		m_I2C_COND.endTransmission();
		if(response.empty())
		{
			m_Initialised = false;
			Logger::error("%s Failed to successfully read from the marine conductivity sensor, check connection!", __PRETTY_FUNCTION__);
		}
	}
	else
	{
		Logger::error("%s Failed to obtain Conductivity I2C file descriptor", __PRETTY_FUNCTION__);
		m_Initialised = false;
	}
	if(m_I2C_PH.init(I2C_ADDRESS_PH))
	{
		m_I2C_PH.beginTransmission();
		std::string response = m_I2C_PH.ASQUERY("I");
		printf("received %s request %02x\n", response.c_str(), I2C_ADDRESS_COND);
		m_I2C_PH.endTransmission();
		if(response.size() == 0)
		{
			m_Initialised = false;
			Logger::error("%s Failed to successfully read from the marine ph sensor, check connection!", __PRETTY_FUNCTION__);
		}
	}
	else
	{
		Logger::error("%s Failed to obtain Ph I2C file descriptor", __PRETTY_FUNCTION__);
		m_Initialised = false;
	}
	m_timer.start();
	return true;
}

void  MarineSensorNode::processMessage(const Message* msg)
{

	MessageType type = msg->messageType();
	if (type == MessageType::DataRequest){
		
		m_timer.sleepUntil(m_miniWaitTime);
		m_timer.reset();
		if (readData(m_temp, m_conductivety, m_ph)){
			m_salidety = Utility::calculateSalidety (m_temp, m_conductivety);
			MessagePtr marineSensorDataMsg = std::make_unique<MarineSensorDataMsg>(m_temp, m_conductivety, m_ph, m_salidety);
			m_MsgBus.sendMessage(std::move(marineSensorDataMsg));
			
		}
		
		
	}
	
	
}




bool MarineSensorNode::readData(float& temperature, float& conductivity, float& ph)
{
	
	
	if(m_Initialised)
	{
		m_I2C_TEMP.beginTransmission();
		std::string TEMP = m_I2C_TEMP.ASQUERY("R");
		m_I2C_TEMP.endTransmission();
		m_timer.sleepUntil(m_miniWaitTime/3);
		m_I2C_COND.beginTransmission();
		std::string COND = m_I2C_COND.ASQUERY("R");
		m_I2C_COND.endTransmission();
		m_timer.sleepUntil(m_miniWaitTime*2/3);
		m_I2C_PH.beginTransmission();
		std::string PH = m_I2C_PH.ASQUERY("R");
		m_I2C_PH.endTransmission();
		
		if(!TEMP.empty())
		{
			temperature = strtof(TEMP.c_str(), NULL);	//convert response to float
		}
		else
		{
			return(false);
		}
		if(!COND.empty())
		{
			conductivity = strtof(COND.c_str(), NULL);	//convert response to float
		}
		else
		{
			return(false);
		}
		if(!PH.empty())
		{
			ph = strtof(PH.c_str(), NULL);				//convert response to float
		}
		else
		{
			return(false);
		}
		return(true);
	}
	else
	{
		Logger::error("%s Cannot read marine sensors as the node was not correctly initialised!", __PRETTY_FUNCTION__);
		return(false);
	}
}




