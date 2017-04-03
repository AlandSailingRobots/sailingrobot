#include "CANPGNReceiver.h"
#include "CANService.h"

CanPGNReceiver::CanPGNReceiver(CANService& service, std::vector<uint32_t> PGNs){
    for(uint32_t pgn : PGNs) {
      service.registerForReading(*this, pgn);
    }
}

CanPGNReceiver::CanPGNReceiver(CANService& service, uint32_t PGN) {
  service.registerForReading(*this, PGN);
}
