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
//	Logger::info("I am in processPGN" + std::to_string(nMsg.PGN));
//	std::cout << nMsg.PGN << std::endl;
  if (nMsg.PGN == 129038 || nMsg.PGN == 129039) {
//	Logger::info("I am in processPGN if pgn = 8/9");
    AISVessel vessel;
    CANAISNode::parsePGN129038_129039(nMsg, vessel);
    m_VesselList.push_back(std::move(vessel));
//    Logger::info("Size after parsePGN129038_129039: " + std::to_string(m_VesselList.size()));
  }
  else if (nMsg.PGN == 129025) {
    parsePGN129025(nMsg);
  }
}

void CANAISNode::parsePGN129038_129039(N2kMsg& nMsg, AISVessel& vessel) {
// void CANAISNode::parsePGN129038_129039(N2kMsg& nMsg) {
  // AISVessel vessel;
  int lat_tmp, lon_tmp, mmsi_test;
  uint16_t cog_tmp, sog_tmp;
  uint8_t byte_arr[4];
//	Logger::info("I am in parsePGN129038_129039");
//	char dat[27];
//	std::memcpy(&dat, &nMsg.Data, sizeof dat);
//	FILE* file = fopen( "myfile.ais", "wb" );
//	fwrite( dat, 1, sizeof dat, file );
//	fclose( file );
 byte_arr[0] = nMsg.Data[1];
 byte_arr[1] = nMsg.Data[2];
 byte_arr[2] = nMsg.Data[3];
 byte_arr[3] = nMsg.Data[4];
 std::memcpy(&mmsi_test, &byte_arr, sizeof mmsi_test);
  vessel.MMSI = ((nMsg.Data[4] << 24) | (nMsg.Data[3] << 16) | (nMsg.Data[2] << 8) | (nMsg.Data[1]));
  lon_tmp = ((nMsg.Data[8] << 24) | (nMsg.Data[7] << 16) | (nMsg.Data[6] << 8) | (nMsg.Data[5]));
  lat_tmp = ((nMsg.Data[12] << 24) | (nMsg.Data[11] << 16) | (nMsg.Data[10] << 8) | (nMsg.Data[9]));
  cog_tmp = ((nMsg.Data[15]) << 8 | (nMsg.Data[14]));
  sog_tmp = ((nMsg.Data[17]) << 8 | (nMsg.Data[16]));
  vessel.latitude = lat_tmp * res_pos;
  vessel.longitude = lon_tmp * res_pos;
  vessel.COG = cog_tmp * res_cog;
  vessel.SOG = sog_tmp * res_sog;
//	Logger::info("MMSI Test:     " + std::to_string(mmsi_test));
//	Logger::info("MMSI:          " + std::to_string(vessel.MMSI));
//	Logger::info("Ves latitude:  " + std::to_string(vessel.latitude));
//	Logger::info("Ves longitude: " + std::to_string(vessel.longitude));
//	Logger::info("Ves cog:       " + std::to_string(vessel.COG));
//	Logger::info("Ves sog:       " + std::to_string(vessel.SOG));

// m_VesselList.push_back(vessel);
}

void CANAISNode::parsePGN129025(N2kMsg& nMsg) {
//  double lat_test, lon_test;
  // uint8_t byte_arr[4];
  int lat_pos,lon_pos; //, lat_tmp, lon_tmp;
  //
  // byte_arr[0] = nMsg.Data[0];
  // byte_arr[1] = nMsg.Data[1];
  // byte_arr[2] = nMsg.Data[2];
  // byte_arr[3] = nMsg.Data[3];
  //
  // std::memcpy(&lat_pos,&byte_arr,sizeof lat_pos);
  // byte_arr[0] = nMsg.Data[4];
  // byte_arr[1] = nMsg.Data[5];
  // byte_arr[2] = nMsg.Data[6];
  // byte_arr[3] = nMsg.Data[7];
  // std::memcpy(&lon_pos,&byte_arr,sizeof lon_pos);
 lat_pos = ((nMsg.Data[3] << 24) | (nMsg.Data[2] << 16) | (nMsg.Data[1] << 8) | (nMsg.Data[0]));
 lon_pos = ((nMsg.Data[7] << 24) | (nMsg.Data[6] << 16) | (nMsg.Data[5] << 8) | (nMsg.Data[4]));

//  lat_test =  lat_tmp * res_pos;
//  lon_test =  lon_tmp * res_pos;

  m_PosLat = lat_pos * res_pos;
  m_PosLon = lon_pos * res_pos;

//    Logger::info(std::to_string(lat_pos));
//	Logger::info(std::to_string(lon_pos));
//    Logger::info("Latiude in processPGN129025: " + std::to_string(m_PosLat));
//    Logger::info("Longiude in processPGN129025: " + std::to_string(m_PosLon));
//  Logger::info("Latiude in processPGN129025 (test): " + std::to_string(lat_test));
//  Logger::info("Longiude in processPGN129025 (test): " + std::to_string(lon_test));
//  Logger::info("Latiude in processPGN129025 (int): " + std::to_string(lat_tmp));
//  Logger::info("Longiude in processPGN129025 (int): " + std::to_string(lon_tmp));
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

  //  Logger::info("Longiude in ThreadFunc: " + std::to_string(node->m_PosLon));
  //  Logger::info("Latiude in ThreadFunc: " + std::to_string(node->m_PosLat));
    if (node->m_VesselList.size() != 0) {
    MessagePtr AISList = std::make_unique<AISDataMsg>(node->m_VesselList, node->m_PosLat, node->m_PosLon);
    node->m_MsgBus.sendMessage(std::move(AISList));
    node->m_lock.unlock();
    node->m_VesselList.clear(); // Should clear the list after a message is sent
    }
    else {
        node->m_lock.unlock();
    }
    timer.sleepUntil(node->m_LoopTime);
    timer.reset();
  }
}
