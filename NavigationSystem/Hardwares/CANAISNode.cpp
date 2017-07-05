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
  AISVessel vessel;
  N2kMsg nMsg;
  IdToN2kMsg(nMsg, msg.id)
  if (IsFastPackage(nMsg)) {
    ParseFastPkg(msg, nMsg);
    if (nMsg.PGN == 129038 || nMsg.PGN == 129039) {
      parsePGN129038_129039(nMsg, vessel);
      m_VesselList.push_back(std::move(vessel));
    }
  }
}

void CANAISNode::processPGN(N2kMsg& nMsg) {

}

void CANAISNode::parsePGN129038_129039(N2kMsg& nMsg, AISVessel vessel) {
  vessel.MMSI = (nMsg.data[4] << 24 | nMsg.data[1]);
  vessel.latitude = (nMsg.data[8] << 24 | nMsg.data[5]);
  vessel.longitude = (nMsg.data[12] << 24 | nMsg.data[9]);
  vessel.COG = (nMsg.data[15] << 8 | nMsg.data[14]);
  vessel.SOG = (nMsg.data[17] << 8 | nMsg.data[16]);
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

    MessagePtr AISList = std::make:unique<AISDataMsg>(node->m_VesselList);
    node->msgBus.sendMessage(std::move(AISList));

    node->m_lock.unlock();

    timer.reset();
  }
}
