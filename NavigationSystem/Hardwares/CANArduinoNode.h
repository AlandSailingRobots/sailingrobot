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

#include "DataBase/DBHandler.h"
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
	CANArduinoNode(MessageBus& messageBus, DBHandler& dbhandler, CANService& canService, double loopTime);
	~CANArduinoNode();
	bool init();
	void processMessage (const Message* message);
	void processFrame (CanMsg& msg);
	void start ();


private:

	static void CANArduinoNodeThreadFunc(ActiveNode* nodePtr);

	///----------------------------------------------------------------------------------
	/// Update values from the database as the loop time of the thread and others parameters
	///----------------------------------------------------------------------------------
	void updateConfigsFromDB();

	const int DATA_OUT_OF_RANGE	= -2000;
	const float MAX_RUDDER_ANGLE = 30;
	const float MAX_WINGSAIL_ANGLE = 13;
	const float INT16_SIZE = 65535;

	float m_RudderFeedback;			//NOTE : degree [-MAX_RUDDER_ANGLE ; MAX_RUDDER_ANGLE]
	float m_WingsailFeedback;			//NOTE : degree [-MAX_WINGSAIL_ANGLE ; MAX_WINGSAIL_ANGLE]
	float m_WindvaneSelfSteerAngle;		//NOTE : info ?
	float m_Radio_Controller_On;		//NOTE : use to define message corresponding to the activity of the RC
	float m_WindvaneActuatorPos;		//NOTE : degree [-? ; ?]
	double m_LoopTime;					// in seconds (ex: 0.5 s)
	DBHandler& m_db;

	std::mutex m_lock;

};
