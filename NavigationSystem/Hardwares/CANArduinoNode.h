#include "Hardwares/CAN_Services/CANService.h"
#include "Hardwares/CAN_Services/CANFrameReceiver.h"
#include "Hardwares/CAN_Services/N2kMsg.h"

#include "MessageBus/Message.h"
#include "MessageBus/MessageBus.h"

class CANArduinoNode : public Node, public CANFrameReceiver {
public:
	CANArduinoNode(MessageBus& messageBus, CANService& canService);
	~CANArduinoNode();	
	bool init();
	void processMessage (const Message* message);
	void processFrame (CanMsg& msg);
};


