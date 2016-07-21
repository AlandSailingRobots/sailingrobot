/****************************************************************************************
 *
 * File:
 * 		ArduinoNode.cpp
 *
 * Purpose:
 *		The Arduino node communicates with the arduino. Sends data about the pressure, rudder, sheet and battery.
 *
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/

#include "ArduinoNode.h"
#include "Messages/ArduinoDataMsg.h"
#include "logger/Logger.h"

// For std::this_thread
#include <chrono>
#include <thread>

#define BLOCK_READ_SIZE 9
#define BLOCK_I2C_ADDRESS_LOC 8

#define DEFAULT_I2C_ADDRESS_PRESSURE 0x07

#define ARDUINO_SENSOR_SLEEP_MS	100


ArduinoNode::ArduinoNode(MessageBus& msgBus)
: ActiveNode(NodeID::Arduino, msgBus), m_Initialised(false)
{
    
}

bool ArduinoNode::init()
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

void ArduinoNode::start()
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

void ArduinoNode::processMessage(const Message* msg)
{

}

void ArduinoNode::ArduinoThreadFunc(void* nodePtr)
{
    ArduinoNode* node = (ArduinoNode*)nodePtr;

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

        MessagePtr msg = std::make_unique<ArduinoDataMsg>(node->m_pressure, node->m_rudder, node->m_sheet, node->m_battery);
        node->m_MsgBus.sendMessage(std::move(msg));
    }
}
