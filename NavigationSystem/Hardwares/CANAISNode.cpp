#include "CANAISNode.h"

CANAISNode::CANAISNode(MessageBus& msgBus, CANService& canService, double loopTime) :
    CANPGNReceiver(canService, {129038, 129039}), ActiveNode(NodeID::CANAIS, msgBus),
    m_VesselList({}), m_LoopTime(loopTime){

}

CANAISNode::~CANAISNode() {

}

bool CANAISNode::init() {
  return true;
}

void CANAISNode::processMessage(const Message* message) {

}

void CANAISNode::processFrame(CanMsg &msg) {
  N2kMsg nMsg;
  IdToN2kMsg(nMsg, msg.id)
  if (IsFastPackage(nMsg)) {
    ParseFastPkg(msg, nMsg);
}
  AISVessel vessel;
  if (nMsg.PGN == 129038) {
    parsePGN129038(nMsg, vessel);
    m_VesselList.push_back(std::move(vessel));
  }
  else if (nMsg.PGN == 129039) {
    parsePGN129039(nMsg, vessel);
    m_VesselList.push_back(std::move(vessel));
  }
}

void CANAISNode::processPGN(N2kMsg& nMsg) {

}

void CANAISNode::parsePGN129038(N2kMsg& nMsg, AISVessel vessel) {
  vessel.MMSI = nMsg.data[1-4]...;
  vessel.latitude = nMsg.data[5-8]...;
  vessel.longitude = nMsg.data[9-12]...;
  vessel.COG = nMsg.data[14-15]...;
  vessel.SOG = nMsg.data[16-17]...;
}

void CANAISNode::parsePGN129039(N2kMsg& nMsg, AISVessel vessel) {
  vessel.MMSI = nMsg.data[1-4]...;
  vessel.latitude = nMsg.data[5-8]...;
  vessel.longitude = nMsg.data[9-12]...;
  vessel.COG = nMsg.data[14-15]...;
  vessel.SOG = nMsg.data[16-17]...;
}

void start() {
  runThread(CANAISNode::CANAISThreadFunc);
}

void CANAISNode::CANAISThreadFunc(ActiveNode* nodePtr) {
  CANAISNode node = dynamic_cast<CANAISNode*> (nodePtr);

  Timer timer;
  timer.start();

  while(true) {
    timer.sleepUntil(node->m_LoopTime*1.0f/1000);
    node->m_lock.lock();
  }
}
