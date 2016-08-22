/****************************************************************************************
 *
 * File:
 * 		XbeeRemote.cpp
 *
 * Purpose:
 *		
 *
 * Developer Notes:
 *
 ***************************************************************************************/


#include "XbeeRemote.h"
#include <stdio.h>
#include "../SystemServices/Logger.h"
#include "../utility/SysClock.h"
#include "../Messages/VesselStateMsg.h"
#include "../Messages/DataRequestMsg.h"
#include "../Messages/WindDataMsg.h"
#include "../Messages/CompassDataMsg.h"
#include "../Messages/GPSDataMsg.h"
#include "../Messages/ActuatorPositionMsg.h"
#include "../Messages/WaypointDataMsg.h"
#include "../Messages/CourseDataMsg.h"

#ifdef __linux__
#include "Network/LinuxSerialDataLink.h"
#include "udpclient.h"
#elif _WIN32
#include "WindowsSerialDataLink.h"
#endif


XbeeRemote*	XbeeRemote::m_Instance = NULL;


#define MONITOR_PORT	4321
#define OTHER_PORT		4322


/***************************************************************************************/
XbeeRemote::XbeeRemote(std::string portName)
	:m_DataLink(NULL), m_Network(NULL), m_PortName(portName),
	 m_LastReceived(0), m_Connected(true), m_Relay({4321, 4322}, "127.0.0.1")
{
	m_Instance = this;
}

/***************************************************************************************/
XbeeRemote::~XbeeRemote()
{

}

/***************************************************************************************/
bool XbeeRemote::initialise()
{
	if(m_PortName.length() == 0)
	{
		Logger::error("%s:%d No port name set!", __FILE__, __LINE__);
		return false;
	}

#ifdef __linux__

	m_DataLink = new LinuxSerialDataLink(m_PortName.c_str(), XBEE_BAUD_RATE);

#elif _WIN32

#endif

	if(m_DataLink->initialise(XBEE_PACKET_SIZE))
	{
		m_Network = new XbeePacketNetwork(*m_DataLink, true);
		m_Network->setIncomingCallback(incomingData);
		return true;
	}
	else
	{
		delete m_DataLink;
		m_DataLink = NULL;
		return false;
	}

}

/***************************************************************************************/
void XbeeRemote::run()
{
	if(m_Network != NULL)
	{
		m_Relay.write("offline=0");

		while(true)
		{
			const int OFFLINE_TIME = 5;
			m_Network->processRadioMessages();

			/* Test
			m_Relay.write("heading=%d gpsHeading=%f speed=%f lat=%.7f lon=%.7f windDir=%f windSpeed=%f rudderAV=%d sailAV=%d pressure=%d currTot=%d", 120, 120., 4., 52.4, 32.6, 120., 4., 4500, 5000, 10, 12);
			 	*/

			if(SysClock::unixTime() - m_LastReceived > OFFLINE_TIME)
			{
				if(m_Connected)
				{
					Logger::info("Lost connected");
					m_Connected = false;
					m_Relay.write("offline=1");
				}

			}
			else if(not m_Connected)
			{
				Logger::info("Gained connection");
				m_Relay.write("offline=0");
				m_Connected = true;
			}
		}
	}
}

/***************************************************************************************/
void XbeeRemote::printMessage(Message* msgPtr, MessageDeserialiser& deserialiser)
{
	m_LastReceived = SysClock::unixTime();

	switch(msgPtr->messageType())
	{
		case MessageType::DataRequest:
		{
			DataRequestMsg msg(deserialiser);

			if(msg.isValid())
			{
				Logger::info("Message Type %s - Source: %d Destination: %d", msgToString(MessageType::DataRequest).c_str(), msg.sourceID(), msg.destinationID());
			}
		}
		break;
		case MessageType::WindData:
		{
			WindDataMsg msg(deserialiser);

			if(msg.isValid())
			{
				Logger::info("Message: %s - Wind Dir: %f Wind Temp: %f Wind Speed: %f", msgToString(MessageType::WindData).c_str(), msg.windDirection(), msg.windTemp(), msg.windSpeed());
			}
		}
		break;
		case MessageType::CompassData:
		{
			CompassDataMsg msg(deserialiser);

			if(msg.isValid())
			{
				Logger::info("Message:  %s - Heading: %d Pitch: %d Roll: %d", msgToString(MessageType::CompassData).c_str(), msg.heading(), msg.pitch(), msg.roll());
			}
		}
		break;
		case MessageType::GPSData:
		{
			GPSDataMsg msg(deserialiser);

			if(msg.isValid())
			{
				Logger::info("Message: %s - Unix Time: %d Lat: %.7lf Lon: %.7lf", msgToString(MessageType::GPSData).c_str(), msg.unixTime(), msg.latitude(), msg.longitude());
			}
		}
		break;
		case MessageType::ActuatorPosition:
		{
			ActuatorPositionMsg msg(deserialiser);

			if(msg.isValid())
			{
				Logger::info("Message: %s - Rudder: %d Sail: %d", msgToString(MessageType::ActuatorPosition).c_str(), msg.rudderPosition(), msg.sailPosition());
			}
		}
		break;
		case MessageType::VesselState:
		{
			VesselStateMsg msg(deserialiser);

			if(msg.isValid())
			{
				unsigned int unixTime = (unsigned)(int)msg.unixTime();
				Logger::info("Message: %s - Unix Time: %u Lat: %.7f Long: %.7f Compass: %d Speed: %f Wind Dir: %f Wind Speed: %f Rudder: %d Sail: %d ", msgToString(MessageType::VesselState).c_str(), unixTime, msg.latitude(), msg.longitude(), msg.compassHeading(), msg.speed(), msg.windDir(), msg.windSpeed(), msg.arduinoRudder(), msg.arduinoSheet());
			}
			else
			{
				Logger::info("Not valid");
			}
		}
		break;
		case MessageType::WaypointData:
		{
			WaypointDataMsg msg(deserialiser);

			if(msg.isValid())
			{
				Logger::info("Message: %s - ID: %d Lat: %.7lf Lon: .7lf", msgToString(MessageType::WaypointData).c_str(), msg.nextId(), msg.nextLatitude(), msg.nextLongitude());
			}
		}
		break;
		case MessageType::CourseData:
		{
			CourseDataMsg msg(deserialiser);

			if(msg.isValid())
			{
				Logger::info("Message: %s - True Wind Dir: %f Distance To WP: %f Course To WP: %f", msgToString(MessageType::CourseData).c_str(), msg.trueWindDir(), msg.distanceToWP(), msg.courseToWP());
			}
		}
		break;
		default:
			Logger::info("Packet: %s", msgToString(msgPtr->messageType()).c_str());
			return;
	}
}

/***************************************************************************************/
void XbeeRemote::sendToUI(Message* msgPtr, MessageDeserialiser& deserialiser)
{
	switch(msgPtr->messageType())
	{
		case MessageType::VesselState:
		{
			VesselStateMsg msg(deserialiser);
			if(msg.isValid())
			{
				m_Relay.write("heading=%d gpsHeading=%f speed=%f lat=%.7f lon=%.7f windDir=%f windSpeed=%f rudderAV=%d sailAV=%d pressure=%d currTot=%d", msg.compassHeading(), msg.gpsHeading(), msg.speed(), msg.latitude(), msg.longitude(), msg.windDir(), msg.windSpeed(), msg.arduinoRudder(), msg.arduinoSheet(), msg.arduinoPressure(), msg.arduinoBattery());
			}
		}
		break;
		case MessageType::WaypointData:
		{
			WaypointDataMsg msg(deserialiser);
			if(msg.isValid())
			{
				m_Relay.write("wpnum=%d ", msg.nextId());
			}
		}
		break;
		case MessageType::CourseData:
		{
			CourseDataMsg msg(deserialiser);
			if(msg.isValid())
			{
				m_Relay.write("twd=%f dtw=%f ctw=%f", msg.trueWindDir(), msg.distanceToWP(), msg.courseToWP());
			}
		}
		break;
		default:
			break;
	}
}

/***************************************************************************************/
 void XbeeRemote::incomingData(uint8_t* data, uint8_t size)
{
	// Decode the message
	MessageDeserialiser deserialiser(data, size);
	Message msg(deserialiser);

	if(msg.isValid())
	{
		deserialiser.resetInternalPtr();

		// Print it to stdout
		m_Instance->printMessage(&msg, deserialiser);

		// Send it to the Monitor UI
		deserialiser.resetInternalPtr();
		m_Instance->sendToUI(&msg, deserialiser);
	}
}
