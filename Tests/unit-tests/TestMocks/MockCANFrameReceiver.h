#pragma once

#include "HardwareServices/CAN_Services/CANService.h"
#include "HardwareServices/CAN_Services/CANFrameReceiver.h"
#include "HardwareServices/CAN_Services/N2kMsg.h"

#include <iostream>

class MockCANReceiver : public CANFrameReceiver {
public:

  MockCANFrameReceiver(CANService& service, std::vector<uint32_t> IDs) : CANFrameReceiver(service, IDs), m_PGNs(IDs)
  {
  }

  void processPGN(CanMsg& msg) {
      IDtoN2k
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
