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
#include "Messages/ArduinoDataMsg.h"
#include "SystemServices/Logger.h"

// For std::this_thread
#include <chrono>
#include <thread>

#define BLOCK_READ_SIZE 9
#define BLOCK_I2C_ADDRESS_LOC 8

#define DEFAULT_I2C_ADDRESS_PRESSURE 0x07

#define ARDUINO_SENSOR_SLEEP_MS	100


ArduinoI2CNode::ArduinoI2CNode(MessageBus& msgBus)
: ActiveNode(NodeID::Arduino, msgBus), m_Initialised(false)
{
    
}

bool ArduinoI2CNode::init()
{
	m_Initialised = false;

    if(m_I2C.init(DEFAULT_I2C_ADDRESS_PRESSURE))
	{
		m_I2C.beginTransmission();

        uint8_t block[BLOCK_READ_SIZE];
        m_I2C.readBlock(block, BLOCK_READ_SIZE);
		uint8_t deviceID = block[BLOCK_I2C_ADDRESS_LOC];

		m_I2C.endTransmission();

		// The Device reports the I2C address that is mentioned in the datasheet
		if(deviceID == DEFAULT_I2C_ADDRESS_PRESSURE)
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

}

void ArduinoI2CNode::ArduinoThreadFunc(void* nodePtr)
{
    ArduinoI2CNode* node = (ArduinoI2CNode*)nodePtr;

    Logger::info("Arduino thread started");

    while(true)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(ARDUINO_SENSOR_SLEEP_MS));

        uint8_t block[BLOCK_READ_SIZE];
        uint16_t reVal;       
        //readValues(block)

        node->m_I2C.beginTransmission();        
            node->m_I2C.readBlock(block, BLOCK_READ_SIZE);
        node->m_I2C.endTransmission();

        reVal = block[0]<<8;
        reVal+=(uint16_t) block[1];
        node->m_pressure = reVal;

        reVal = block[2]<<8;
        reVal+=(uint16_t) block[3];
        node->m_rudder = reVal;

        reVal = block[4]<<8;
        reVal+=(uint16_t) block[5];
        node->m_sheet = reVal;

        reVal = block[6]<<8;
        reVal+=(uint16_t) block[7];
        node->m_battery = reVal;

        ArduinoDataMsg* msg = new ArduinoDataMsg(node->m_pressure, node->m_rudder, node->m_sheet, node->m_battery);
        node->m_MsgBus.sendMessage(msg);
    }
}

void ArduinoI2CNode::sendToArduino(uint32_t dataID, uint32_t CANID, uint8_t* CANData)
{
	//TODO
}

void ArduinoI2CNode::loadOutgoingMessage(const Message* message)
{
	if(message->messageType() == MessageType::ActuatorPosition)
	{
		ActuatorPositionMsg* msg = (ActuatorPositionMsg*)message;

		if (nodeID() == NodeID::CANBus)
		{
			//Get relevant command
			m_rudderPosition = msg->rudderPosition();
			m_sailPosition = msg->sailPosition();
		
		}
		
		else
		{
			Logger::warning("%s CANBus: %d Unknown/Unregistered CANBus message NodeID", __PRETTY_FUNCTION__, (int)nodeID());
		}
		
		m_actuatorPositionsReady = true;
		
	}
	/*
	Not implemented but will probably be like this...

	elseif(message->messageType() == MessageType::WindVaneCommand)
	{
		WindVaneCommandMsg* msg = (WindVaneCommandMsg*)message;

		if (nodeID() == NodeID::CANBus)
		{
			//Get relevant command
			m_windVaneCommand = msg->windVaneCommand();
		
		}
		
		else
		{
			Logger::warning("%s Actuator: %d Unknown/Unregistered actuator message NodeID", __PRETTY_FUNCTION__, (int)nodeID());
		}

		m_windVaneCommandReady = true;
		
	}
	*/
}

void ArduinoI2CNode::passToCANBus()
{
	if (m_actuatorPositionsReady)
	{
		uint8_t  CANData[8]
		uint32_t CANID = CANID::ActuatorCommandsOnly;

		CANData[0]= m_rudderPosition >> 8;
		CANData[1]= m_rudderPosition & 0x00ff;
		CANData[2]= m_sailPosition >> 8;
		CANData[3]= m_sailPosition & 0x00ff;

		if (m_windVaneCommandReady)
		{
			CANData[4]= m_windVaneCommand;
			CANID = CANID::ActuatorandWindVaneCommands;
		}

		sendToArduino(DataID::CANBus, CANID, CANData); //TODO
	
		m_actuatorPositionsReady = false;
		m_windVanceCommandReady = false;

	}
		
	else
	{
		Logger::warning("%s CANbus: %d Attempted to pass CAN commands with no actuator commands loaded.", __PRETTY_FUNCTION__, (int)nodeID());
	}
}


void ArduinoI2CNode::processIncomingCAN(uint32_t CANID, uint8_t* CANData)
{
	if(CANID == CANID::ActuatorFeedbackOnly)
	{
		uint16_t = rudderPosition = (CANData[0] << 8) | CANData[1];
		uint16_t = sailPosition = (CANData[2] << 8) | CANData[3];		

		ActuatorPositionMsg *actuatorMsg = new ActuatorPositionMsg(rudderPosition, sailPosition, true);
		m_MsgBus.sendMessage(actuatorMsg);
	}
	
	else if (CANID == CANID::ActuatorAndWindFeedback)
	{
		uint16_t = rudderPosition = (CANData[0] << 8) | CANData[1];
		uint16_t = sailPosition = (CANData[2] << 8) | CANData[3];
		uint8_t = windDir = //TODO;
		uint8_t = windSpeed = //TODO;
		uint16_t = windTemp = //TODO;

		ActuatorPositionMsg *actuatorMsg = new ActuatorPositionMsg(rudderCommand_norm, sailCommand_norm, true);
		m_MsgBus.sendMessage(actuatorMsg);

		WindDataMsg *windDataMsg = new WindDataMsg(windDir, windSpeed, windTemp);
		m_MsgBus.sendMessage(windDataMsg);
	}
}
