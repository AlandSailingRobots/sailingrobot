/****************************************************************************************
 *
 * File:
 * 		ArduinoNodeForWindSensor.cpp
 *
 * Purpose:
 *		The Arduino node communicates with the arduino. Sends data about the pressure, rudder, sheet and battery.
 *
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/

#include "ArduinoNodeForWindSensor.h"
#include "Messages/WindDataMsg.h"
#include "SystemServices/Logger.h"

// For std::this_thread
#include <chrono>
#include <thread>

#define BLOCK_READ_SIZE 9
#define BLOCK_I2C_ADDRESS_LOC 8

#define DEFAULT_I2C_ADDRESS_PRESSURE 0x07

#define ARDUINO_SENSOR_SLEEP_MS	100


ArduinoNodeForWindSensor::ArduinoNodeForWindSensor(MessageBus& msgBus)
: ActiveNode(NodeID::Arduino, msgBus), m_Initialised(false)
{

}

bool ArduinoNodeForWindSensor::init()
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

void ArduinoNodeForWindSensor::start()
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

void ArduinoNodeForWindSensor::processMessage(const Message* msg)
{

}

void ArduinoNodeForWindSensor::ArduinoThreadFunc(void* nodePtr)
{
    ArduinoNodeForWindSensor* node = (ArduinoNodeForWindSensor*)nodePtr;

    Logger::info("Arduino thread started");

    while(true)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(ARDUINO_SENSOR_SLEEP_MS));

        uint8_t block[BLOCK_READ_SIZE],remainder,multiple;
        //readValues(block)

        node->m_I2C.beginTransmission();
            node->m_I2C.readBlock(block, BLOCK_READ_SIZE);
        node->m_I2C.endTransmission();

        remainder = (uint8_t)block[0];
        multiple = (uint8_t)block[1];
        node->m_windDirection = (remainder+multiple*255)*(360/1362.0);
	if(node->m_windDirection > 360){
		node->m_windDirection=360;
	}
        MessagePtr msg = std::make_unique<WindDataMsg>(node->m_windDirection, 0, 0);
        node->m_MsgBus.sendMessage(std::move(msg));
    }
}