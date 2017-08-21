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
#include "SystemServices/Timer.h"


GPSDNode::GPSDNode(MessageBus& msgBus, DBHandler& dbhandler, double loopTime)
	: ActiveNode(NodeID::GPS, msgBus), m_Initialised(false), m_GpsConnection(0),
	m_Lat(0), m_Lon(0), m_Speed(0), m_Course(0),m_LoopTime(loopTime),m_db(dbhandler)
{
	msgBus.registerNode(*this, MessageType::ServerConfigsReceived);
}

GPSDNode::~GPSDNode()
{
	delete m_GpsConnection;
}

bool GPSDNode::init()
{
	m_Initialised = false;
	updateConfigsFromDB();
	m_GpsConnection = new gpsmm("localhost", DEFAULT_GPSD_PORT);
	m_currentDay = SysClock::day();

	if (m_GpsConnection->stream(WATCH_ENABLE | WATCH_JSON) != NULL) {
		m_Initialised = true;
	}
	else
	{
		Logger::warning("Is GPSD running?");
	}

	return m_Initialised;
}

void GPSDNode::updateConfigsFromDB()
{
	m_LoopTime = m_db.retrieveCellAsDouble("config_gps","1","loop_time");
}

void GPSDNode::processMessage(const Message* msgPtr)
{
	MessageType type = msgPtr->messageType();
	if( type == MessageType::ServerConfigsReceived)
	{
			updateConfigsFromDB();
	}
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

void GPSDNode::GPSThreadFunc(ActiveNode* nodePtr)
{
	GPSDNode* node = dynamic_cast<GPSDNode*> (nodePtr);

	Logger::info("GPSD thread started");

	Timer timer;
	timer.start();
	while(true)
	{
		// Controls how often we pump out messages
		timer.sleepUntil(node->m_LoopTime);

		if(not node->m_GpsConnection->waiting(node->GPS_TIMEOUT_MICRO_SECS))
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

		unixTime = newData->fix.time;

		//Once a day: make sure system clock is updated from gps to ensure accuracy
		if (gps_online){
			int today = SysClock::day();
			if (node->m_currentDay != today){
				SysClock::setTime(unixTime);
				node->m_currentDay = SysClock::day();
			}
		}

		node->m_Lat = newData->fix.latitude;
		node->m_Lon = newData->fix.longitude;
		node->m_Speed = newData->fix.speed;
		node->m_Course = newData->fix.track;

		int satCount = 0;
		satCount = newData->satellites_used;

		GPSMode mode = GPSMode::NoUpdate;
		mode = static_cast<GPSMode>(newData->fix.mode);

		MessagePtr msg = std::make_unique<GPSDataMsg>(gps_hasFix, gps_online, node->m_Lat, node->m_Lon, unixTime, node->m_Speed, node->m_Course, satCount, mode);
		node->m_MsgBus.sendMessage(std::move(msg));
		timer.reset();
	}
}
