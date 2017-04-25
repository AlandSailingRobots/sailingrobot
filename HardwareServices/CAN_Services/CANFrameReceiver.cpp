#include "CANFrameReceiver.h"
#include "CANService.h"

CANFrameReceiver::CANFrameReceiver(CANService& service, std::vector<uint32_t> IDs){
    for(uint32_t id : IDs) {
      service.registerForReading(*this, id);
    }
}

CANFrameReceiver::CANFrameReceiver(CANService& service, uint32_t ID) {
  service.registerForReading(*this, ID);
}
