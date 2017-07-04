#include "CANAISNode.h"

CANAISNode::CANAISNode(MessageBus& msgBus, CANService& canService, double loopTime) :
    ActiveNode(NodeID::CANAIS, msgBus), m_MMSI(0), m_Lat(0), m_Lon(0), m_COG(0), m_SOG(0){

}

CANAISNode::~CANAISNode() {

}

bool CANAISNode::init() {
  return true;
}

void CANAISNode::processMessage(const Message* message) {

}

void CANAISNode::processFrame(CanMsg &msg) {

}

void CANAISNode::processPGN(N2kMsg& nMsg) {

}

void start() {
  
}
