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
	//if (Logger::init()) {
	//	Logger::info("I am in processPGN");
	//7}
	std::cout << nMsg.PGN << std::endl;
  if (nMsg.PGN == 129038 || nMsg.PGN == 129039) {
		if (Logger::init()) {
		Logger::info("I am in processPGN if pgn = 8/9");
	}
    AISVessel vessel;
    parsePGN129038_129039(nMsg, vessel);
    m_VesselList.push_back(std::move(vessel));
  }
  else if (nMsg.PGN == 129025) {

    parsePGN129025(nMsg);
  }
}

void CANAISNode::parsePGN129038_129039(N2kMsg& nMsg, AISVessel& vessel) {
	if (Logger::init("AISTest.log")) {
		Logger::info("I am in processPGN129038_129039");
	}
	char dat[27];
	std::memcpy(&dat, &nMsg.Data, sizeof dat);
	FILE* file = fopen( "myfile.ais", "wb" );
	fwrite( dat, 1, sizeof dat, file );
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
	std::cout << std::endl << vessel.MMSI << std::endl;
	std::cout << vessel.latitude << std::endl;
	std::cout << vessel.longitude << std::endl;
	std::cout << vessel.COG << std::endl;
	std::cout << vessel.SOG << std::endl << std::endl;
}

void CANAISNode::parsePGN129025(N2kMsg& nMsg) {
	
  uint8_t byte_arr[4];
  int lat_pos,lon_pos;
	// uint16_t test;
  byte_arr[0] = nMsg.Data[0];
  byte_arr[1] = nMsg.Data[1];
  byte_arr[2] = nMsg.Data[2];
  byte_arr[3] = nMsg.Data[3];
	//test = ((nMsg.Data[1] << 8 | nMsg.Data[0]));
  std::memcpy(&lat_pos,&byte_arr,sizeof lat_pos);
  byte_arr[0] = nMsg.Data[4];
  byte_arr[1] = nMsg.Data[5];
  byte_arr[2] = nMsg.Data[6];
  byte_arr[3] = nMsg.Data[7];
  std::memcpy(&lon_pos,&byte_arr,sizeof lon_pos);
  m_PosLat = lat_pos;
  m_PosLon = lon_pos;
	
/* 	std::cout << nMsg.Data[0] << std::endl;
	std::cout << std::endl << test << std::endl;
	std::cout << lat_pos << std::endl; */
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

    MessagePtr AISList = std::make_unique<AISDataMsg>(node->m_VesselList, node->m_PosLat, node->m_PosLon);
    node->m_MsgBus.sendMessage(std::move(AISList));
    node->m_lock.unlock();

    timer.sleepUntil(node->m_LoopTime);
    timer.reset();
  }
}
