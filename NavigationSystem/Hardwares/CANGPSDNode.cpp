#include "CANGPSDNode.h"

CANGPSDNode::CANGPSDNode(MessageBus& msgBus, double loopTime)
	: ActiveNode(NodeID::GPS, msgBus), m_initialised, m_GpsConnection(0),
	m_Lat(0), m_Lon(0), m_Speed(0), m_Heading(0),m_LoopTime(loopTime)
{

}

CANGPSDNode::~CANGPSDNODE () {

}

bool CANGPSDNode::init() {
	m_initialised = true;
	return m_initialised;
}

void CANGPSDNode::processMessage (const Message* message) {

}

void CANGPSDNode::processFrame (CanMsg& msg) {
	
}

void CANGPSDNode::start() {
	runThread(CANGPSDNode::CANGPSDThreadFunc);
}

void CANGPSDNode::CANGPSDThreadFunc(ActiveNode* nodePtr) {
	CANGPSDNode* node = dynamic_cast<CANGPSDNode*> (NodePtr);
	Logger::info("CANPGPSD started");
	Timer timer;
	timer.start();

	while(true) {
		timer.sleepUntil(node->m_LoopTime*1.0f / 1000);
		node->m_lock.lock();


	}
}
