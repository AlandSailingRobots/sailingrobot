/****************************************************************************************
*
* File:
* 		CANArduinoNode.h
*
* Purpose:
*		 Process messages from the arduino in the Actuatorunit via the CAN-Service including ActuatorFeedback and RC status . 
*
* Developer Notes:
*		 The CAN frame id numbers that this node subscribes to are:
*			701, 702, (more to be added later?)
*
***************************************************************************************/
#include <chrono>
#include <thread>
#include <mutex>
#include <vector>
#include <iostream>

#include "Hardwares/CAN_Services/CANService.h"
#include "Hardwares/CAN_Services/CANFrameReceiver.h"
#include "Hardwares/CAN_Services/N2kMsg.h"
#include "MessageBus/ActiveNode.h"
#include "MessageBus/Message.h"
#include "MessageBus/MessageBus.h"
#include "Messages/ASPireActuatorFeedbackMsg.h"
#include "SystemServices/Timer.h"
#include "Math/Utility.h"


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

	const int DATA_OUT_OF_RANGE	= -2000;
	const float MAX_RUDDER_ANGLE = 30;
	const float MAX_WINGSAIL_ANGLE = 13;
	const float INT16_SIZE = 65535;
	
	float m_RudderFeedback;
	float m_WingsailFeedback; 
	float m_WindvaneSelfSteerAngle;
	bool  m_RadioControllerOn;
	float m_WindvaneActuatorPos;
	int   m_loopTime;
	std::mutex m_lock;
	
};	



