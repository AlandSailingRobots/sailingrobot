#include "WindStateNode.h"
#include "Math/Utility.h"

#include <vector>

WindStateNode::WindStateNode(MessageBus& msgBus, const int twd)
: Node(NodeID::WindStateNode, msgBus), m_twdSize(twd)
{
  msgBus.registerNode(*this, MessageType::StateMessage);
  msgBus.registerNode(*this, MessageType::WindData);
}

bool WindStateNode::init(){
  return true;
}

// If we have not yet received the necessary messages
// we do not have the information to calculate what we need to.
// Should maybe be done in a nicer way.

void WindStateNode::processMessage(const Message* message){
  MessageType type = message->messageType();

  if(type == MessageType::StateMessage){
    m_stateMsgReceived = true;
    parseStateMessage((StateMessage*) message);

    if(!m_windDataReceived){
      return;
    }
    updateApparentWind();
    sendMessage();
  }

  else if(type == MessageType::WindData){
    m_windDataReceived = true;
    parseWindMessage((WindDataMsg*) message);

    if(!m_stateMsgReceived){
      return;
    }
    updateTrueWind();
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

void WindStateNode::sendMessage() {

  std::lock_guard<std::mutex> lockGuard(m_Lock);

  MessagePtr windState = std::make_unique<WindStateMsg>(m_trueWindSpeed, m_trueWindDirection,
                                                    m_apparentWindSpeed, m_apparentWindDirection);
  m_MsgBus.sendMessage(std::move(windState));
}


void WindStateNode::updateApparentWind() {
  Utility::calculateApparentWind(m_WindDir, m_WindSpeed, m_vesselSpeed, m_vesselHeading,
                                     m_trueWindDirection, m_apparentWindSpeed, m_apparentWindDirection);
}

void WindStateNode::updateTrueWind() {
  std::vector<float> twdBuffer(m_twdSize);

  m_trueWindDirection = Utility::getTrueWindDirection(m_WindDir, m_WindSpeed, m_vesselSpeed, m_vesselHeading, twdBuffer, m_twdSize);
  m_trueWindSpeed     = Utility::calculateTrueWindSpeed(m_WindDir, m_WindSpeed, m_vesselSpeed, m_vesselHeading);
}