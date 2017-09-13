/****************************************************************************************
 *
 * File:
 * 		HMC6343Node.cp
 *
 * Purpose:
 *		The HMC6343 node communicates with a HMC6343 compass and provides CompassDataMsgs
 *		to the message bus.
 *
 * Developer Notes:
 * 		I2C Address
 *
 * 		The I2C address is different from what is mentioned in the datasheet. The
 * 		datasheet mentions it is 0x32, however when accessing the device you need to use
 * 		0x19 for successful communications. The register on the HMC6343 that contains the
 * 		I2C address returns 0x32, the address mentioned in the datasheet.
 *
 *
 ***************************************************************************************/

#include "HMC6343Node.h"
#include "Messages/CompassDataMsg.h"
#include "SystemServices/Logger.h"
#include "wiringPi.h"
#include "Math/Utility.h"
#include "SystemServices/Timer.h"


// For std::this_thread
#include <chrono>
#include <thread>


// The device datasheet mentions that this is 0x32. In our case this doesn't seem to be
// correct, so stick with 0x19.
#define I2C_ADDRESS 			0x19
#define I2C_DATASHEET_ADDRESS	0x32

// HMC6343 Commands
#define COM_POST_HEADING 		0x50
#define COM_POST_TILT 			0x55
#define COM_POST_MAG 			0x45
#define COM_POST_ACCEL 			0x40
#define COM_READ_EEPROM 		0xE1

#define EEPROM_ADDRESS			0x00


#define COM_ORIENT_LEVEL 0x72
#define COM_ORIENT_SIDEWAYS 0x73
#define COM_ORIENT_FLATFRONT 0x74





HMC6343Node::HMC6343Node(MessageBus& msgBus, DBHandler& dbhandler)
: ActiveNode(NodeID::Compass, msgBus), m_Initialised(false), m_HeadingBufferSize(1),
m_LoopTime(0.5), m_db(dbhandler)
{

}


bool HMC6343Node::init()
{
	m_Initialised = false;
	updateConfigsFromDB();

	if(m_I2C.init(I2C_ADDRESS))
	{
		m_I2C.beginTransmission();
		m_I2C.writeReg(COM_READ_EEPROM, EEPROM_ADDRESS);
		delay(10);
		uint8_t deviceID = m_I2C.I2Cread();
		printf("received %02x request %02x\n",deviceID,I2C_ADDRESS);

		m_I2C.endTransmission();

		// The Device reports the I2C address that is mentioned in the datasheet
		if(deviceID == I2C_DATASHEET_ADDRESS)
		{
			m_Initialised = true;
		}
		else
		{
			Logger::error("%s Failed to successfully read from the HMC6343 compass, check connection!", __PRETTY_FUNCTION__);
		}
	}
	else
	{
		Logger::error("%s Failed to obtain I2C file descriptor", __PRETTY_FUNCTION__);
	}

	return m_Initialised;
}


void HMC6343Node::start()
{
	if(m_Initialised)
	{
		runThread(HMC6343ThreadFunc);
	}
	else
	{
		Logger::error("%s Cannot start compass sensor thread as the node was not correctly initialised!", __PRETTY_FUNCTION__);
	}
}

void HMC6343Node::updateConfigsFromDB()
{
	m_LoopTime = m_db.retrieveCellAsDouble("config_compass","1","loop_time");
	m_HeadingBufferSize = m_db.retrieveCellAsInt("config_compass","1","heading_buffer_size");
}

void HMC6343Node::processMessage(const Message* msg)
{
	if( msg->messageType() == MessageType::ServerConfigsReceived)
	{
			updateConfigsFromDB();
	}
}


bool HMC6343Node::readData(float& heading, float& pitch, float& roll)
{
	const int BYTES_TO_READ = 6;
	uint8_t buffer[BYTES_TO_READ];

	if(m_Initialised)
	{
		// Extract the data from the compass, each piece of data is made up of two bytes, and their are
		// 3 pieces of data.
		m_I2C.beginTransmission();
		m_I2C.I2Cwrite(COM_POST_HEADING);

		delay(1);

		int val = 0;
		for(int i = 0; i < BYTES_TO_READ; i++)
		{
			val = m_I2C.I2Cread();

			if(val != -1)
			{
				buffer[i] = static_cast<uint8_t>(val);
			}
			else
			{
				m_I2C.endTransmission();
				return false;
			}
		}

		m_I2C.endTransmission();

		// The data is stretched across two separate bytes in big endian format
		heading = (static_cast<int16_t>((buffer[0] << 8) + buffer[1])) / 10.f;
		pitch = (static_cast<int16_t>((buffer[2] << 8) + buffer[3])) / 10.f;
		roll = (static_cast<int16_t>((buffer[4] << 8) + buffer[5])) / 10.f;

		return true;
	}
	else
	{
		Logger::error("%s Cannot read compass as the node was not correctly initialised!", __PRETTY_FUNCTION__);
		return false;
	}
}

bool HMC6343Node::setOrientation(CompassOrientation orientation)
{
	if(m_Initialised)
	{
		m_I2C.beginTransmission();
		m_I2C.I2Cwrite((uint8_t)orientation);
		m_I2C.endTransmission();
		return true;
	}
	else
	{
		Logger::error("%s Cannot set compass orientation as the node was not correctly initialised!", __PRETTY_FUNCTION__);
		return false;
	}
}

void HMC6343Node::calibrate(int calibrationTime){
	Timer calTimer;
	calTimer.start ();
	calTimer.reset();
	Logger::info("Started calibration");
	m_I2C.beginTransmission();
	m_I2C.I2Cwrite((uint8_t)113);
	calTimer.sleepUntil(calibrationTime);
	m_I2C.I2Cwrite((uint8_t)126);
	m_I2C.endTransmission();
	Logger::info("Calibration finished");
	calTimer.stop();
}

void HMC6343Node::HMC6343ThreadFunc(ActiveNode* nodePtr)
{
	const int MAX_ERROR_COUNT = 100;

	HMC6343Node* node = dynamic_cast<HMC6343Node*> (nodePtr);

	Logger::info("HMC6343 compass thread started!");
	unsigned int errorCount = 0;
	std::vector<float> headingData(node->m_HeadingBufferSize);
	int headingIndex = 0;

	Timer timer;
	timer.start();
	while(true)
	{

		if(errorCount >= MAX_ERROR_COUNT)
		{
			errorCount = 0;
			Logger::error("%s Failed to read the compass %d times", __PRETTY_FUNCTION__, MAX_ERROR_COUNT);
		}

		float heading, pitch, roll;
		if(node->readData(heading, pitch, roll))
		{
			errorCount = 0;

			// Store the heading data
			if(headingData.size() < (uint16_t)node->m_HeadingBufferSize)
			{
				headingData.push_back(heading);
			}
			else
			{
				if(headingIndex >= node->m_HeadingBufferSize)
				{
					headingIndex = 0;
				}

				headingData[headingIndex] = heading;
				headingIndex++;
			}
			// Post the data to the message bus
			MessagePtr msg = std::make_unique<CompassDataMsg>(int(Utility::meanOfAngles(headingData) + 0.5), pitch, roll);
			node->m_MsgBus.sendMessage(std::move(msg));
		}
		else
		{
			errorCount++;
		}
		// Controls how often we pump out messages
		timer.sleepUntil(node->m_LoopTime);
		timer.reset();
	}
}
