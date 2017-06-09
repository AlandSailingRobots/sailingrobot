#pragma once

#include "HardwareServices/CAN_Services/CANService.h"
#include "HardwareServices/CAN_Services/CANFrameReceiver.h"
#include "HardwareServices/CAN_Services/N2kMsg.h"

#include <iostream>

class MockCANReceiver : public CANFrameReceiver {
public:

  MockCANReceiver(CANService& service, std::vector<uint32_t>& IDs) : CANFrameReceiver(service, IDs), m_IDs(IDs)
  {
  }

  void processFrame(CanMsg& msg)
  {
    uint32_t ID = msg.id;
    if(ID == m_IDs[0])
    {
      got_message = true;
    }
  }

  bool message_received()
  {
    return got_message;
  }

private:
  const std::vector<uint32_t> m_IDs;
  bool got_message = false;
};
