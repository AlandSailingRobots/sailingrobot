#pragma once

#include "HardwareServices/CAN_Services/CANService.h"
#include "HardwareServices/CAN_Services/CANPGNReceiver.h"
#include "HardwareServices/CAN_Services/N2kMsg.h"

#include <iostream>

class MockCANReceiver : public CANPGNReceiver {
public:

  MockCANReceiver(CANService& service, std::vector<uint32_t> PGNs) : CANPGNReceiver(service, PGNs), m_PGNs(PGNs)
  {
    if(PGNs[0] == 1304){
      N2kMsg msg;
      msg.PGN = 1337;
      service.sendN2kMessage(msg);
    }
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
