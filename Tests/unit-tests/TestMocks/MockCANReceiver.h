#pragma once

#include "../../../SystemServices/CANPGNReceiver.h"
#include "../../../SystemServices/CANService.h"
#include "../../../SystemServices/N2kMsg.h"

#include <iostream>

class MockCANReceiver : public CANPGNReceiver {
public:

  MockCANReceiver(CANService& service, std::vector<uint32_t> PGNs) : CANPGNReceiver(service, PGNs), m_PGNs(PGNs)
  {
  }

  void processPGN(N2kMsg& msg) {
    uint32_t PGN = msg.PGN;
    if(PGN == m_PGNs[0]){
      got_message = true;
    }
  }

  bool message_received(){
    return got_message;
  }

private:
  const std::vector<uint32_t> m_PGNs;
  bool got_message = false;
};