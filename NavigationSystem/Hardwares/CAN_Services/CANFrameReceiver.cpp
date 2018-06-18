#include "CANFrameReceiver.h"
#include "CANService.h"
#include "SystemServices/Logger.h"
#include "CanBusCommon/CanMessageHandler.h"

#include <iostream>

CANFrameReceiver::CANFrameReceiver(CANService& service, std::vector<uint32_t> IDs){
    for(uint32_t id : IDs) {
      service.registerForReading(*this, id);
    }
}

CANFrameReceiver::CANFrameReceiver(CANService& service, uint32_t ID) {
  service.registerForReading(*this, ID);
}

void CANFrameReceiver::processFrameAndLogErrors(CanMsg &msg) {
	CanMessageHandler messageHandler(msg);

	if(messageHandler.getErrorMessage() != NO_ERRORS) {
		Logger::error("Error in Can Message with id:%d   Error message code:%d\n",messageHandler.getMessageId(),messageHandler.getErrorMessage());
	}
	processFrame(msg);
}
