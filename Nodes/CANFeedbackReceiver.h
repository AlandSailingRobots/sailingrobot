#include "HardwareServices/CAN_Services/CANService.h"
#include "HardwareServices/CAN_Services/CANFrameReceiver.h"
#include "HardwareServices/CAN_Services/N2kMsg.h"

#include "Messages/Message.h"
#include "Messages/MessageTypes.h"
#include "MessageBus/MessageBus.h"


class CANFeedbackReceiver : public Node, public CANFrameReceiver {
public:
    CANFeedbackReceiver(MessageBus& messageBus, CANService& canService);
    ~CANFeedbackReceiver();
    bool init();
    void processMessage(const Message* message);
    void processFrame(CanMsg& msg);
};