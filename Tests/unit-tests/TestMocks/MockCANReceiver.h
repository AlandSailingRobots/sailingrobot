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
    std::cout << "Message received at PGN : " << msg.PGN << std::endl;
  }

private:
  std::vector<uint32_t> PGNs = {1304, 1307};

}