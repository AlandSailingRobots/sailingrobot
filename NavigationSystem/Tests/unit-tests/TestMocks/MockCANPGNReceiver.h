#pragma once

#include "Hardwares/CAN_Services/CANPGNReceiver.h"
#include "Hardwares/CAN_Services/CANService.h"
#include "Hardwares/CAN_Services/N2kMsg.h"

#include <iostream>

class MockCANPGNReceiver : public CANPGNReceiver {
   public:
    // MockCANPGNReceiver::MockCANPGNReceiver(CANService& service, uint32_t PGN) :
    //     CANPGNReceiver(service, PGN), m_PGN(PGN) {}

    MockCANPGNReceiver::MockCANPGNReceiver(CANService& service, std::vector<uint32_t> PGNs)
        : CANPGNReceiver(service, PGNs), m_PGNs(PGNs) {}

    void processPGN(N2kMsg& nMsg) {
        uint32_t PGN = nMsg.PGN;
        for (auto pgns : m_PGNs) {
            if (PGN == pgns) {
                got_message = true;
            }
        }
    }

    bool message_received() { return got_message; }

   private:
    bool got_message = false;
    std::vector<uint32_t> m_PGNs;
};
