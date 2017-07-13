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
  std::cout << "HEJ" << std::endl;
}

bool CANAISNode::init() {
  return true;
}

void CANAISNode::processMessage(const Message* message) {

}

void CANAISNode::processPGN(N2kMsg& nMsg) {
  if (nMsg.PGN == 129038 || nMsg.PGN == 129039) {
    AISVessel vessel;
    parsePGN129038_129039(nMsg, vessel);
    m_VesselList.push_back(std::move(vessel));
  }
}

void CANAISNode::parsePGN129038_129039(N2kMsg& nMsg, AISVessel& vessel) {
  vessel.MMSI = ((nMsg.Data[4] << 24) | (nMsg.Data[3] << 16) | (nMsg.Data[2] << 8) | (nMsg.Data[1]));
  uint8_t byte_arr[4];
  float lat,lon;
  byte_arr[0] = nMsg.Data[5];
  byte_arr[1] = nMsg.Data[6];
  byte_arr[2] = nMsg.Data[7];
  byte_arr[3] = nMsg.Data[8];
  std::memcpy(&lat,&byte_arr,sizeof lat);
  byte_arr[0] = nMsg.Data[9];
  byte_arr[1] = nMsg.Data[10];
  byte_arr[2] = nMsg.Data[11];
  byte_arr[3] = nMsg.Data[12];
  std::memcpy(&lon,&byte_arr,sizeof lon);
  vessel.latitude = lat;
  vessel.longitude = lon;
  vessel.COG = ((nMsg.Data[15]) << 8 | (nMsg.Data[14]));
  vessel.SOG = ((nMsg.Data[17]) << 8 | (nMsg.Data[16]));
}

void CANAISNode::start() {
  runThread(CANAISNode::CANAISThreadFunc);
}

void CANAISNode::CANAISThreadFunc(ActiveNode* nodePtr) {
  CANAISNode* node = dynamic_cast<CANAISNode*> (nodePtr);
  bool PRINT = false;
  Timer timer;
  timer.start();

  while(true) {

    node->m_lock.lock();

    MessagePtr AISList = std::make_unique<AISDataMsg>(node->m_VesselList);
    node->m_MsgBus.sendMessage(std::move(AISList));
    node->m_lock.unlock();
    if (PRINT) {
      if (node->m_VesselList.size() > 0) {
        std::cout << std::endl << node->m_VesselList[0].MMSI << std::endl;
        std::cout << node->m_VesselList[0].latitude << std::endl;
        std::cout << node->m_VesselList[0].longitude << std::endl;
        std::cout << node->m_VesselList[0].COG << std::endl;
        std::cout << node->m_VesselList[0].SOG << std::endl;
      }
    }
    timer.sleepUntil(node->m_LoopTime*1.0f/1000);
    timer.reset();
  }
}
