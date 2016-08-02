/****************************************************************************************
 *
 * File:
 * 		GPSDNode.cpp
 *
 * Purpose:
 *		A GPSD node that uses the GPSD library to provide GPS data to the message bus.
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/

#include "GPSDNode.h"
#include "SystemServices/Logger.h"
#include "Messages/GPSDataMsg.h"


#define GPS_SENSOR_SLEEP_MS			50
#define GPS_TIMEOUT_MICRO_SECS		50000000


GPSDNode::GPSDNode(MessageBus& msgBus)
	: ActiveNode(NodeID::GPS, msgBus), m_Initialised(false), m_GpsConnection(0), m_Lat(0), m_Lon(0), m_Speed(0), m_Heading(0)
{

}

GPSDNode::~GPSDNode()
{
	delete m_GpsConnection;
}

bool GPSDNode::init()
{
	m_Initialised = false;
	m_GpsConnection = new gpsmm("localhost", DEFAULT_GPSD_PORT);

	if (m_GpsConnection->stream(WATCH_ENABLE | WATCH_JSON) != NULL) {
		m_Initialised = true;
	}
	else
	{
		Logger::warning("Is GPSD running?");
	}

	return m_Initialised;
}

void GPSDNode::processMessage(const Message* msgPtr)
{

}

void GPSDNode::start()
{
	if(m_Initialised)
	{
		runThread(GPSThreadFunc);
	}
	else
	{
		Logger::error("%s Cannot start GPSD sensor thread as the node was not correctly initialised!", __PRETTY_FUNCTION__);
	}
}

void GPSDNode::GPSThreadFunc(void* nodePtr)
{
	GPSDNode* node = (GPSDNode*)nodePtr;

	Logger::info("GPSD thread started");

	while(true)
	{
		// Controls how often we pump out messages
		std::this_thread::sleep_for(std::chrono::milliseconds(GPS_SENSOR_SLEEP_MS));

		if(not node->m_GpsConnection->waiting(GPS_TIMEOUT_MICRO_SECS))
		{
			Logger::warning("%s GPSD read time out!", __PRETTY_FUNCTION__);
			continue;
		}

		struct gps_data_t* newData;
		if ((newData = node->m_GpsConnection->read()) == NULL)
		{
			Logger::error("%s GPSD read error out!", __PRETTY_FUNCTION__);
			continue;
		}

		// Get status flags
		unsigned long int flags = newData->set;

		bool gps_online =  (flags & ( 1 << 4 )) >> 4; //flags & ONLINE_SET;
		bool gps_hasFix = (newData->status > 0);
		double unixTime = 0;

		//if(flags & TIME_SET)
		{
			unixTime = newData->fix.time;
		}

		//if(flags & LATLON_SET)
		{
			node->m_Lat = newData->fix.latitude;
			node->m_Lon = newData->fix.longitude;
		}

		//if(flags & SPEED_SET)
		{
			node->m_Speed = newData->fix.speed;
		}

		// if(flags & TRACK_SET)
		{
			node->m_Heading = newData->fix.track;
		}

		int satCount = 0;
		// if(flags & SATELLITE_SET)
		// {
			satCount = newData->satellites_used;
		//}

		GPSMode mode = GPSMode::NoUpdate;
		//if(flags & MODE_SET)
		{
			mode = static_cast<GPSMode>(newData->fix.mode);
		}

		MessagePtr msg = std::make_unique<GPSDataMsg>(gps_hasFix, gps_online, node->m_Lat, node->m_Lon, unixTime, node->m_Speed, node->m_Heading, satCount, mode);
		node->m_MsgBus.sendMessage(std::move(msg));

	}
}
