#include "CANSolarTrackerNode.h"

CANSolarTrackerNode::CANSolarTrackerNode(MessageBus& msgBus, double loopTime)
	: ActiveNode(NodeID::GPS, msgBus), m_initialised, m_GpsConnection(0),
	m_Lat(0), m_Lon(0), m_Time(0), m_Heading(0),m_LoopTime(loopTime)
{

}

CANSolarTrackerNode::~CANSolarTrackerNode () {

}

bool CANSolarTrackerNode::init() {
	m_initialised = true;
	return m_initialised;
}

void CANSolarTrackerNode::processMessage (const Message* message) {

}

void CANSolarTrackerNode::processFrame (N2kmsg& msg) {
	if(msg.PGN == 129025) {
			m_Lat = msg.Data | ;
			m_Lon = msg.Data >> 32;
	}
}

void CANSolarTrackerNode::start() {
	runThread(CANSolarTrackerNode::CANSolarTrackerThreadFunc);
}

void CANSolarTrackerNode::CANGSolarTrackerThreadFunc(ActiveNode* nodePtr) {
	CANSolarTrackerNode* node = dynamic_cast<CANSolarTrackerNode*> (NodePtr);

	Timer timer;
	timer.start();

	while(true) {
		timer.sleepUntil(node->m_LoopTime*1.0f / 1000);
		node->m_lock.lock();


	}
}
