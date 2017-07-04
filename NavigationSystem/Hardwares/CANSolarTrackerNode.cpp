#include "CANSolarTrackerNode.h"

CANSolarTrackerNode::CANSolarTrackerNode(MessageBus& msgBus, CANService& canService, double loopTime)
	: ActiveNode(NodeID::CANSolarTracker, msgBus), m_initialised, m_GpsConnection(0),
	m_Lat(0), m_Lon(0), m_Time(0), m_Heading(0),m_LoopTime(loopTime)
{
	m_Lat = DATA_OUT_OF_RANGE;
	m_Lon = DATA_OUT_OF_RANGE;
	m_Heading = DATA_OUT_OF_RANGE;

	msgBus.registerNode(*this, MessageType::StateMessage);
}

CANSolarTrackerNode::~CANSolarTrackerNode () {

}

bool CANSolarTrackerNode::init() {
	m_initialised = true;
	return m_initialised;
}

void CANSolarTrackerNode::processMessage (const Message* message) {
	if (message->messageType() == MessageType::StateMessage) {
		m_Heading = message->heading();
		m_Lat = message->latitude();
		m_Lon = message->longitude();
	}
}

void CANSolarTrackerNode::processFrame ( CanMsg& msg ) {

}

void CANSolarTrackerNode::start() {
	runThread(CANSolarTrackerNode::CANSolarTrackerThreadFunc);
}

void CANSolarTrackerNode::CANSolarTrackerThreadFunc(ActiveNode* nodePtr) {
	CANSolarTrackerNode* node = dynamic_cast<CANSolarTrackerNode*> (nodePtr);

	Timer timer;
	timer.start();

	while(true) {
		timer.sleepUntil(node->m_LoopTime*1.0f / 1000);
		node->m_lock.lock();

		m_Time = STEADY_CLOCK::now;

		if (node->m_Lat == node->DATA_OUT_OF_RANGE &&
				node->m_Lon == node-> DATA_OUT_OF_RANGE &&
				node->m_Heading == node->DATA_OUT_OF_RANGE) {
			m_lock.unlock();
			continue;
		}
		MessagePtr solarMsg = std::make_unique<SolarDataMsg>(node->m_Lat, node->m_Lon, node->m_heading, node->m_Time);
		node -> m_MsgBus.sendMessage(std::move(solarMsg));

		node->m_lock.unlock();

		timer.reset();
	}
}
