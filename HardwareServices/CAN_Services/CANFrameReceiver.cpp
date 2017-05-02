#include "CANFrameReceiver.h"
#include "CANService.h"

#include <iostream>

CANFrameReceiver::CANFrameReceiver(CANService& service, std::vector<uint32_t> IDs){
    for(uint32_t id : IDs) {
      std::cout << "in CANFrameReceiver ";
      service.registerForReading(*this, id);
      std::cout << "after service.registerForReading";
    }
}

CANFrameReceiver::CANFrameReceiver(CANService& service, uint32_t ID) {
  service.registerForReading(*this, ID);
}
