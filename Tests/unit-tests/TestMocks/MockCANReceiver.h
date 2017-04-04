#pragma once

#include "../../../SystemServices/CANPGNReceiver.h"
#include "../../../SystemServices/CANService.h"
#include "../../../SystemServices/N2kMsg.h"

#include <iostream>

class MockCANReceiver : public CANPGNReceiver {
public:

  MockCANReceiver(CANService& service) : CANPGNReceiver(service, PGNs)
  {  }

  void processPGN(N2kMsg& msg) {
    uint32_t PGN = msg.PGN;
    if(PGN == PGNs[0]){
      got_message = true;
    }
  }

  bool message_received(){
    return got_message;
  }

private:
  std::vector<uint32_t> PGNs = {1304, 1307};
  bool got_message = false;
};