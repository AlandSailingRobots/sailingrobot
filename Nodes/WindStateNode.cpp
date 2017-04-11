#include "WindStateNode.h"

WindStateNode::WindStateNode(MessageBus& msgBus)
: Node(NodeID::WindStateNode, msgBus)
{
  msgBus.registerNode(*this, MessageType::StateMessage);
  msgBus.registerNode(*this, MessageType::WindData);
}

bool WindStateNode::init(){
  return true;
}

void WindStateNode::processMessage(const Message* message){
  MessageType type = message->messageType();

  if(type == MessageType::StateMessage){
    parseStateMessage((StateMessage*) message);
    sendMessage();
  }
  else if(type == MessageType::WindData){
    parseWindMessage((WindDataMsg*) message);
  }

}

void WindStateNode::parseStateMessage(StateMessage* msg) {
  m_vesselHeading = msg->heading();
  m_vesselLat     = msg->latitude();
  m_vesselLon     = msg->longitude();
  m_vesselSpeed   = msg->speed();
}

void WindStateNode::parseWindMessage(WindDataMsg* msg) {
  m_WindDir       = msg->windDirection();
  m_WindSpeed     = msg->windSpeed();
  m_WindTemp      = msg->windTemp();
}