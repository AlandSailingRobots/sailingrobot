#include "Hardwares/CAN_Services/CANService.h"
#include "Hardwares/CAN_Services/CANFrameReceiver.h"
#include "Hardwares/CAN_Services/N2kMsg.h"
#include "MessageBus/ActiveNode.h"
#include "MessageBus/Message.h"
#include "MessageBus/MessageBus.h"
#include "SystemServices/Timer.h"
#include "Math/Utility.h"

#include <mutex>
#include <vector>
#include <iostream>


class CANArduinoNode : public ActiveNode, public CANFrameReceiver {
public:
	CANArduinoNode(MessageBus& messageBus, CANService& canService, int time_filter_ms);
	~CANArduinoNode();	
	bool init();
	void processMessage (const Message* message);
	void processFrame (CanMsg& msg);
	void start ();

	
private:
	
	static void CANArduinoNodeThreadFunc(ActiveNode* nodePtr);

	const int DATA_OUT_OF_RANGE	=	-2000;
	const float MAX_RUDDER_ANGLE = 30;
	const float MAX_WINGSAIL_ANGLE = 13;
	const float INT16_SIZE = 65535;
	
	float m_RudderFeedback;
	float m_WingsailFeedback; 
	float m_WindvaneSelfSteerAngle;
	float m_Radio_Controller_On;
	float m_WindvaneActuatorPos;
	int   m_loopTime;
	std::mutex m_lock;
	
};	



