#pragma once

class CANService;

#include "N2kMsg.h"

#include <stdint.h>
#include <vector>

class CANPGNReceiver {
   public:
    CANPGNReceiver(CANService& service, std::vector<uint32_t> PGNs);
    CANPGNReceiver(CANService& service, uint32_t PGN);

    virtual void processPGN(N2kMsg& msg) = 0;
};
