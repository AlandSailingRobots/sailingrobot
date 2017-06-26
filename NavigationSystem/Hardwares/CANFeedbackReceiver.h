#include "Hardwares/CAN_Services/CANService.h"
#include "Hardwares/CAN_Services/CANFrameReceiver.h"
#include "Hardwares/CAN_Services/N2kMsg.h"

#include "MessageBus/Message.h"
#include "MessageBus/MessageBus.h"


class CANFeedbackReceiver : public Node, public CANFrameReceiver {
public:
    CANFeedbackReceiver(MessageBus& messageBus, CANService& canService, int time_filter_ms);
    ~CANFeedbackReceiver();
    bool init();
    void processMessage(const Message* message);
    void processFrame(CanMsg& msg);
};
