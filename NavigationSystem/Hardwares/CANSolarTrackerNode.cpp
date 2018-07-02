/****************************************************************************************
*
* File:
* 		CANSolarTrackerNode.cpp
*
* Purpose:
*		Sends the position, heading and time to the solar tracker
*
*
* Developer Notes:
*		As of right now, only unit test exist, uncertain if it should work like this
*		Since we don't have the solartracker yet I dont know exactly what messages to
*		send but it is easily changed later
*
***************************************************************************************/
#include "CANSolarTrackerNode.h"

const int DATA_OUT_OF_RANGE = -2000;

CANSolarTrackerNode::CANSolarTrackerNode(MessageBus& msgBus, DBHandler& dbhandler, CANService& canService, double loopTime)
	: ActiveNode(NodeID::CANSolarTracker, msgBus), m_CANService(&canService), m_LoopTime(loopTime), m_db(dbhandler)
{
	m_Lat = DATA_OUT_OF_RANGE;
	m_Lon = DATA_OUT_OF_RANGE;
	m_Heading = DATA_OUT_OF_RANGE;

	msgBus.registerNode(*this, MessageType::StateMessage);
	msgBus.registerNode(*this, MessageType::ServerConfigsReceived);
}

CANSolarTrackerNode::~CANSolarTrackerNode () {

}

bool CANSolarTrackerNode::init() {
	updateConfigsFromDB();
	return true;
}

void CANSolarTrackerNode::updateConfigsFromDB(){
	m_LoopTime = m_db.tableColumnDouble("config_solar_tracker", "loop_time", "1");
}

void CANSolarTrackerNode::processMessage (const Message* message) {
	MessageType type = message->messageType();
	if ( type == MessageType::StateMessage) {
		StateMessage* stateMsg = (StateMessage*) message;
		m_Heading = stateMsg->heading();
		m_Lat = stateMsg->latitude();
		m_Lon = stateMsg->longitude();
		time_t rawtime;
		struct tm * timeinfo;

		time ( &rawtime );
		timeinfo = localtime ( &rawtime );

		m_Hour = timeinfo->tm_hour;
		m_Minute = timeinfo->tm_min;
	}
	else if (type == MessageType::ServerConfigsReceived){
		updateConfigsFromDB();
	}
}

void CANSolarTrackerNode::processFrame ( CanMsg& msg ) {

}

void CANSolarTrackerNode::sendMsg (float lat, float lon, float head, uint16_t h, uint16_t m) {
	CanMsg Cmsg, Cmsg2;
	Cmsg.id = 703;
	Cmsg.header.ide = 0;
	Cmsg.header.length = 8;
	Cmsg2.id = 704;
	Cmsg2.header.ide = 0;
	Cmsg2.header.length = 8;
	uint8_t latBytes[4];
	uint8_t lonBytes[4];
	uint8_t headingBytes[4];


	std::memcpy(&latBytes, &lat, sizeof latBytes);
	std::memcpy(&lonBytes, &lon, sizeof lonBytes);
	Cmsg.data[0] = latBytes[0];
	Cmsg.data[1] = latBytes[1];
	Cmsg.data[2] = latBytes[2];
	Cmsg.data[3] = latBytes[3];
	Cmsg.data[4] = lonBytes[0];
	Cmsg.data[5] = lonBytes[1];
	Cmsg.data[6] = lonBytes[2];
	Cmsg.data[7] = lonBytes[3];

	std::memcpy(&headingBytes, &head, sizeof headingBytes);
	Cmsg2.data[0] = headingBytes[0];
	Cmsg2.data[1] = headingBytes[1];
	Cmsg2.data[2] = headingBytes[2];
	Cmsg2.data[3] = headingBytes[3];
	Cmsg2.data[4] = h & 0xff;
	Cmsg2.data[5] = h >> 8;
	Cmsg2.data[6] = m & 0xff;
	Cmsg2.data[7] = m >> 8;

	m_CANService->sendCANMessage(Cmsg);
	m_CANService->sendCANMessage(Cmsg2);
}

void CANSolarTrackerNode::start() {
	runThread(CANSolarTrackerNode::CANSolarTrackerThreadFunc);
}

void CANSolarTrackerNode::CANSolarTrackerThreadFunc(ActiveNode* nodePtr) {
	CANSolarTrackerNode* node = dynamic_cast<CANSolarTrackerNode*> (nodePtr);
	node->m_Hour = 0;
	node->m_Minute = 0;

	Timer timer;
	timer.start();

	while(true) {

		node->m_lock.lock();

		if ( not ((node->m_Lat == DATA_OUT_OF_RANGE) || (node->m_Lon == DATA_OUT_OF_RANGE) ||
			(node->m_Heading == DATA_OUT_OF_RANGE)))
		{
			node->sendMsg(node->m_Lat, node->m_Lon, node->m_Heading, node->m_Hour, node->m_Minute);
		}

		node->m_lock.unlock();

		timer.sleepUntil(node->m_LoopTime);
		timer.reset();
	}
}
