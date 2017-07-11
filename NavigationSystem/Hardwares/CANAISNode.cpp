#include "CANAISNode.h"

CANAISNode::CANAISNode(MessageBus& msgBus, CANService& canService, double loopTime) :
    CANPGNReceiver(canService, {129038, 129039}), ActiveNode(NodeID::CANAIS, msgBus),
    m_VesselList({}), m_LoopTime(loopTime){
      AISVessel v1;
      v1.MMSI = 0;
      v1.latitude = 0;
      v1.longitude = 0;
      v1.COG = 0;
      v1.SOG = 0;
      m_VesselList.push_back(v1);
}

CANAISNode::~CANAISNode() {

}

bool CANAISNode::init() {
  return true;
}

void CANAISNode::processMessage(const Message* message) {

}


void CANAISNode::processPGN(N2kMsg& nMsg) {
  // if (nMsg.PGN == 129038 || nMsg.PGN == 129039) {
  //   AISVessel vessel;
  //   parsePGN129038_129039(nMsg, vessel);
  //   m_VesselList.push_back(std::move(vessel));
  // }
}

void CANAISNode::parsePGN129038_129039(N2kMsg& nMsg, AISVessel& vessel) {
  // vessel.MMSI = (nMsg.Data[4] << 24 | nMsg.Data[1]);
  // vessel.latitude = (nMsg.Data[8] << 24 | nMsg.Data[5]);
  // vessel.longitude = (nMsg.Data[12] << 24 | nMsg.Data[9]);
  // vessel.COG = (nMsg.Data[15] << 8 | nMsg.Data[14]);
  // vessel.SOG = (nMsg.Data[17] << 8 | nMsg.Data[16]);
}

void CANAISNode::start() {
  runThread(CANAISNode::CANAISThreadFunc);
}

void CANAISNode::CANAISThreadFunc(ActiveNode* nodePtr) {
  CANAISNode* node = dynamic_cast<CANAISNode*> (nodePtr);

  Timer timer;
  timer.start();

  while(true) {

    node->m_lock.lock();

    // Maybe send a can messge instead, but it is to long?
    std::cout << std::endl << node->m_VesselList.size() << std::endl;
    MessagePtr AISList = std::make_unique<AISDataMsg>(node->m_VesselList);
    node->m_MsgBus.sendMessage(std::move(AISList));
    node->m_lock.unlock();

    timer.sleepUntil(node->m_LoopTime*1.0f/1000);
    timer.reset();
  }
}
