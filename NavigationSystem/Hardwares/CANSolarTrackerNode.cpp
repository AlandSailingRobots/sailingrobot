#include "CANSolarTrackerNode.h"

CANSolarTrackerNode::CANSolarTrackerNode(MessageBus& msgBus, CANService& canService, double loopTime)
	: CANFrameReceiver(canService, 700), ActiveNode(NodeID::CANSolarTracker, msgBus), m_LoopTime(loopTime)
{
	// m_Lat = DATA_OUT_OF_RANGE;
	// m_Lon = DATA_OUT_OF_RANGE;
	// m_Heading = DATA_OUT_OF_RANGE;
	// m_Time = DATA_OUT_OF_RANGE;

	msgBus.registerNode(*this, MessageType::StateMessage);
}

CANSolarTrackerNode::~CANSolarTrackerNode () {

}

bool CANSolarTrackerNode::init() {
	// m_initialised = true;
	return true;
}

void CANSolarTrackerNode::processMessage (const Message* message) {
	if (message->messageType() == MessageType::StateMessage) {
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
}

void CANSolarTrackerNode::processFrame ( CanMsg& msg ) {

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
		timer.sleepUntil(node->m_LoopTime*1.0f / 1000);
		node->m_lock.lock();

		if (node->m_Lat == node->DATA_OUT_OF_RANGE &&
				node->m_Lon == node-> DATA_OUT_OF_RANGE &&
				node->m_Heading == node->DATA_OUT_OF_RANGE) {
			node->m_lock.unlock();
			continue;
		}
		MessagePtr solarMsg = std::make_unique<SolarDataMsg>(node->m_Lat, node->m_Lon, node->m_Heading, node->m_Hour, node->m_Minute);
		node -> m_MsgBus.sendMessage(std::move(solarMsg));

		node->m_lock.unlock();

		timer.reset();
	}
}
