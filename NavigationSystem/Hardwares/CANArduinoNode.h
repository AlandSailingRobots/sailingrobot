#include "Hardwares/CAN_Services/CANService.h"
#include "Hardwares/CAN_Services/CANFrameReceiver.h"
#include "Hardwares/CAN_Services/N2kMsg.h"

#include "MessageBus/Message.h"
#include "MessageBus/MessageBus.h"

class CANArduinoNode : public Node, public CANFrameReceiver {
public:
	CANArduinoNode(MessageBus& messageBus, CANService& canService, int time_filter_ms);
	~CANArduinoNode();	
	bool init();
	void processMessage (const Message* message);
	void processFrame (CanMsg& msg);
	
private:
	
	float m_RudderFeedback;
  float m_WingsailFeedback; 
  float m_WindvaneSelfSteerAngle;

	std::mutex m_lock;
	
	const int DATA_OUT_OF_RANGE	=	-2000;
};


