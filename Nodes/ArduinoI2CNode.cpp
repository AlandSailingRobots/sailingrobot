/****************************************************************************************
 *
 * File:
 * 		ArduinoI2CNode.cpp
 *
 * Purpose:
 *		The Arduino node communicates with the arduino. Sends data about the pressure, rudder, sheet and battery.
 *
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/

#include "ArduinoI2CNode.h"
#include "SystemServices/Logger.h"
#include "Messages/WindDataMsg.h"
#include "Messages/ArduinoDataMsg.h"
#include "Messages/ActuatorPositionMsg.h"
#include "Messages/DataTransferIDs.h"

// For std::this_thread
#include <chrono>
#include <thread>

#define BLOCK_READ_SIZE 9
#define DATA_ID_LOC 0
#define INIT_ID 0xFF
#define READ_COMMAND 0

#define ARDUINO_ADDRESS 0x07

#define ARDUINO_SENSOR_SLEEP_MS	100


ArduinoI2CNode::ArduinoI2CNode(MessageBus& msgBus)
: ActiveNode(NodeID::Arduino, msgBus), m_Initialised(false)
{
	msgBus.registerNode(this,MessageType::ActuatorCommand);
	msgBus.registerNode(this,MessageType::WindVaneCommand);
}

bool ArduinoI2CNode::init()
{
	m_Initialised = false;

    if(m_I2C.init(ARDUINO_ADDRESS))
	{
		m_I2C.beginTransmission();

        uint8_t block[BLOCK_READ_SIZE];
        m_I2C.readBlock(block, INIT_ID);
		uint8_t initID = block[DATA_ID_LOC];

		m_I2C.endTransmission();

		// The Device reports the I2C address that is mentioned in the datasheet
		if(initID == INIT_ID)
		{
			m_Initialised = true;
		}
		else
		{
			Logger::error("%s Failed to successfully read from the Arduino, check connection!", __PRETTY_FUNCTION__);
		}
	}
	else
	{
		Logger::error("%s Failed to obtain I2C file descriptor", __PRETTY_FUNCTION__);
	}

	return m_Initialised;
}

void ArduinoI2CNode::start()
{
	if(m_Initialised)
	{
		runThread(ArduinoThreadFunc);
	}
	else
	{
		Logger::error("%s Cannot start Aurdino thread as the node was not correctly initialised!", __PRETTY_FUNCTION__);
	}
}


void ArduinoI2CNode::processMessage(const Message* msg)
{
	m_mutex.lock();
	m_locked = true;

	if(msg->messageType() == MessageType::ActuatorCommand)
	{
		ActuatorPositionMsg* actuatorMsg = (ActuatorPositionMsg*)msg;

		if (nodeID() == NodeID::CANBus)
		{
			//Get relevant command
			m_rudderCommand = actuatorMsg->rudderPosition();
			m_sailCommand = actuatorMsg->sailPosition();
		
		}
		
		else
		{
			Logger::warning("%s CANBus: %d Unknown/Unregistered CANBus message NodeID", __PRETTY_FUNCTION__, (int)nodeID());
		}
		
		m_actuatorCommandsReady = true;
		
	}
	/*
	Not implemented but will probably be like this...

	elseif(msg->messageType() == MessageType::WindVaneCommand)
	{
		WindVaneCommandMsg* windVaneMsg = (WindVaneCommandMsg*)msg;

		if (nodeID() == NodeID::CANBus)
		{
			//Get relevant command
			m_windVaneCommand = windVaneMsg->windVaneCommand();
		
		}
		
		else
		{
			Logger::warning("%s Actuator: %d Unknown/Unregistered actuator message NodeID", __PRETTY_FUNCTION__, (int)nodeID());
		}

		m_windVaneCommandReady = true;
		
	}
	*/
	m_locked = false;
	m_mutex.unlock();
}


void ArduinoI2CNode::ArduinoThreadFunc(void* nodePtr)
{
    ArduinoI2CNode* node = (ArduinoI2CNode*)nodePtr;

    Logger::info("Arduino thread started");

    while(true)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(ARDUINO_SENSOR_SLEEP_MS));

        uint8_t data[BLOCK_READ_SIZE];       
        //readValues(block)

        node->m_I2C.beginTransmission();        
        node->m_I2C.readBlock(data, READ_COMMAND);
        node->m_I2C.endTransmission();

        node->processI2CData(data);

        if (node->m_actuatorCommandsReady)
        {
        	node->m_mutex.lock();
			node->m_locked = true;
        	
        	node->passToCANBus();

        	node->m_locked = false;
			node->m_mutex.unlock();
        }
    }
}


int ArduinoI2CNode::sendToArduino(uint8_t* data, uint8_t dataID)
{
	uint8_t size = DataIDToByteLength(dataID);
	m_I2C.beginTransmission();
	int retVal = m_I2C.writeBlock(data, size, dataID);
	m_I2C.endTransmission();

	return retVal;
}

void ArduinoI2CNode::passToCANBus()
{
	if (m_actuatorCommandsReady)
	{
		uint8_t  CANData[8];
		uint32_t DataID = DataID::ActuatorCommands;

		CANData[0]= m_rudderCommand >> 8;
		CANData[1]= m_rudderCommand & 0x00ff;
		CANData[2]= m_sailCommand >> 8;
		CANData[3]= m_sailCommand & 0x00ff;

		if (m_windVaneCommandReady)
		{
			CANData[4]= m_windVaneCommand;
			DataID = DataID::ActuatorAndWindVaneCommands;
		}

		sendToArduino(CANData, DataID); //TODO
	
		m_actuatorCommandsReady = false;
		m_windVaneCommandReady = false;

	}
		
	else
	{
		Logger::warning("%s ArduinoI2C: %d Attempted to pass CAN commands with no actuator commands loaded.", __PRETTY_FUNCTION__, (int)nodeID());
	}
}

void ArduinoI2CNode::processI2CData(uint8_t* data)
{
	uint8_t dataID = data[DATA_ID_LOC];
	int i = 1;

	if (dataID & DataID::ActuatorFeedback)
	{
		uint16_t rudderPosition = (data[i+0] << 8) | data[i+1];
		uint16_t sailPosition = (data[i+2] << 8) | data[i+3];		

		ActuatorPositionMsg *actuatorMsg = new ActuatorPositionMsg(rudderPosition, sailPosition, true);
		m_MsgBus.sendMessage(actuatorMsg);

		i+= 4;
	}
	
	if (dataID & DataID::WindSensorFeedback)
	{
		uint16_t windDir = 0;//TODO;
		uint8_t windSpeed = 0;//TODO;
		uint8_t windTemp = 0;//TODO;

		WindDataMsg *windDataMsg = new WindDataMsg(windDir, windSpeed, windTemp);
		m_MsgBus.sendMessage(windDataMsg);

		i+= 4;
	}
	
	if (dataID & DataID::AnalogFeedback)
	{
		uint16_t pressure = (data[i+0] << 8) | data[i+1];
		uint16_t battery = (data[i+2] << 8) | data[i+3];

		ArduinoDataMsg *arduinoMsg = new ArduinoDataMsg(pressure, battery);
		m_MsgBus.sendMessage(arduinoMsg);

		i+= 4;
	}

}