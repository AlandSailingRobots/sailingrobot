/*Handels incoming data from Ardunin */

#include "CANArduinoNode.h"
#include "Messages/ArduinoDataMsg.h"

CANArduinoNode::CANArduinoNode(MessageBus& messageBus, CANService& canService) : Node(NodeID::CANArduino, messageBus), CANFrameReceiver(canService, 702)

{   }

CANArduinoNode::~CANArduinoNode(){

}

bool CANArduinoNode::init() {
	return true;
}

void CANArduinoNode::processMessage (const Message* message){

}

void CANArduinoNode::processFrame (CanMsg& msg) {
	if (msg.id == 702) {
		uint16_t RC = (msg.data[1] << 8 | msg.data[0]);
		
		MessagePtr arduinoMsg = std::make_unique<ArduinoDataMsg>(0,0,0,0,RC);
		m_MsgBus.sendMessage(std::move(arduinoMsg));
	}
}
