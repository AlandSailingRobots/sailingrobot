#include "CANPGNReceiver.h"
#include "CANService.h"

CANPGNReceiver::CANPGNReceiver(CANService& service, std::vector<uint32_t> PGNs){
    for(uint32_t pgn : PGNs) {
      service.registerForReading(*this, pgn);
    }
}

CANPGNReceiver::CANPGNReceiver(CANService& service, uint32_t PGN) {
  service.registerForReading(*this, PGN);
}
