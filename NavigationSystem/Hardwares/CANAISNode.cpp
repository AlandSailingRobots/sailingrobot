#include "CANAISNode.h"

CANAISNode::CANAISNode(MessageBus& msgBus, CANService& canService, double loopTime) :
    CANPGNReceiver(canService, {129025, 129038, 129039}), ActiveNode(NodeID::CANAIS, msgBus),
    m_VesselList({}), m_PosLat(0), m_PosLon(0), m_LoopTime(loopTime){

}

CANAISNode::~CANAISNode() {

}

bool CANAISNode::init() {
  return true;
}

void CANAISNode::processMessage(const Message* message) {

}

void CANAISNode::processPGN(N2kMsg& nMsg) {
  switch (nMsg.PGN) {
    case 129038:
    case 129039:
      CANAISNode::parsePGN129038_129039(nMsg);
      break;
    case 129025:
      CANAISNode::parsePGN129025(nMsg);
      break;
    case 129794:
      CANAISNode::parsePGN129794(nMsg);
      break;
    case 129810:
      CANAISNode::parsePGN129810(nMsg);
      break;
    default:
      break;
  }
}

void CANAISNode::parsePGN129038_129039(N2kMsg& nMsg) {
  /*
  * To know which bytes to pick out check the spreadsheet on the drive (Software/CAN/Messages)
  * Multiply the integers with their resolution to get the right numbers
  */
  AISVessel vessel;
  int lon_tmp, lat_tmp, cog_tmp, sog_tmp;

  vessel.MMSI = ((nMsg.Data[4] << 24) | (nMsg.Data[3] << 16) | (nMsg.Data[2] << 8) | (nMsg.Data[1]));
  lon_tmp = ((nMsg.Data[8] << 24) | (nMsg.Data[7] << 16) | (nMsg.Data[6] << 8) | (nMsg.Data[5]));
  lat_tmp = ((nMsg.Data[12] << 24) | (nMsg.Data[11] << 16) | (nMsg.Data[10] << 8) | (nMsg.Data[9]));
  cog_tmp = ((nMsg.Data[15]) << 8 | (nMsg.Data[14]));
  sog_tmp = ((nMsg.Data[17]) << 8 | (nMsg.Data[16]));
  vessel.latitude = lat_tmp * res_pos;
  vessel.longitude = lon_tmp * res_pos;
  vessel.COG = cog_tmp * res_cog;
  vessel.SOG = sog_tmp * res_sog;

  m_VesselList.push_back(vessel);
}

void CANAISNode::parsePGN129025(N2kMsg& nMsg) {
  /*
  * To know which bytes to pick out check the spreadsheet on the drive (Software/CAN/Messages)
  * Multiply the integers with their resolution to get the right numbers
  */
  int lat_pos,lon_pos;

  lat_pos = ((nMsg.Data[3] << 24) | (nMsg.Data[2] << 16) | (nMsg.Data[1] << 8) | (nMsg.Data[0]));
  lon_pos = ((nMsg.Data[7] << 24) | (nMsg.Data[6] << 16) | (nMsg.Data[5] << 8) | (nMsg.Data[4]));

  m_PosLat = lat_pos * res_pos;
  m_PosLon = lon_pos * res_pos;
}

void CANAISNode::parsePGN129794(N2kMsg& nMsg) {
  uint16_t len_tmp, beam_tmp;

  vessel.MMSI = ((nMsg.Data[4] << 24) | (nMsg.Data[3] << 16) | (nMsg.Data[2] << 8) | (nMsg.Data[1]));
  len_tmp = ((nMsg.Data[38] << 8) | (nMsg.Data[37]));
  beam_tmp = ((nMsg.Data[40] << 8) | (nMsg.Data[39]));

  m_Length = len_tmp * res_size;
  m_Beam = beam_tmp * res_size;
}

void CANAISNode::parsePGN129810(N2kMsg& nMsg) {
  uint16_t len_tmp, beam_tmp;

  vessel.MMSI = ((nMsg.Data[4] << 24) | (nMsg.Data[3] << 16) | (nMsg.Data[2] << 8) | (nMsg.Data[1]));
  len_tmp = ((nMsg.Data[21] << 8) | (nMsg.Data[20]));
  beam_tmp = ((nMsg.Data[23] << 8) | (nMsg.Data[22]));

  m_Length = len_tmp * res_size;
  m_Beam = beam_tmp * res_size;
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
    if (node->m_VesselList.size() != 0) {
      MessagePtr AISList = std::make_unique<AISDataMsg>(node->m_VesselList, node->m_PosLat, node->m_PosLon);
      node->m_MsgBus.sendMessage(std::move(AISList));
      node->m_lock.unlock();
      node->m_VesselList.clear();
    }
    else {
        node->m_lock.unlock();
    }
    timer.sleepUntil(node->m_LoopTime);
    timer.reset();
  }
}
