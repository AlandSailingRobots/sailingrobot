/****************************************************************************************
 *
 * File:
 * 		PowerTrackNode.h
 *
 * Purpose:
 *      Collects voltage and current data from the actuators, ECUs, and solar
 *      panel to track the power, sends it to the logger.
 *		
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/

 #include "PowerTrackNode.h"

 #include <chrono>
 #include <thread>

 #include "../Messages/PowerStateMsg.h"
 #include "../Math/CourseMath.h"
 #include "../SystemServices/Logger.h"
 #include "../SystemServices/Timer.h"


 PowerTrackNode::PowerTrackNode(MessageBus& msgBus, DBhandler& dbhandler, double loopTime)
    : ActiveNode(NodeID::PowerTrack, msgBus),
            m_ArduinoPressure(0), m_ArduinoRudder(0), m_ArduinoSheet(0), m_ArduinoBattery(0),
            m_CurrentDataCurrent(0), m_CurrentDataVoltage(0), m_CurrentDataElement(0),
            m_Looptime(looptime), m_db(dbhandler)
{
	msgBus.registerNode(*this, MessageType::ArduinoData);
	msgBus.registerNode(*this, MessageType::CurrentSensorData);
}

//----------------------------------------------------------------------------------
PowerTrackNode::~PowerTrackNode()
{
	server.shutdown();
}

//----------------------------------------------------------------------------------
bool PowerTrackNode::init()
{&
	return server.start( 9600 );
}

void PowerTrackNode::start()
{
	runThread(PowerTrackThreadFunc);
}

void PowerTrackNode::processMessage(const *Message msg)
{
	MessageType type = msg->messageType();

	switch(type)
	{
		case MessageType::ArduinoData:
			processArduinoMessage((ArduinoDataMsg*) msg);
			break;

		case MessageType::CurrentSensorData:
			processCurrentSensorData((CurrentSensorDataMsg*) msg);
			break;

		default:
			return;
	}
}

void PowerTrackNode::processArduinoMessage(ArduinoDataMsg* msg)
{
	m_ArduinoPressure = msg->pressure();
	m_ArduinoRudder = msg->rudder();
	m_ArduinoSheet = msg->sheet();
	m_ArduinoBattery = msg->battery();
}

void PowerTrackNode::processCurrentSensorData(CurrentSensorDataMsg* msg)
{
	m_CurrentDataCurrent = msg->current();
	m_CurrentDataVoltage = msg->voltage();
	m_CurrentDataElement = msg->element();
}

void PowerTrackNode::PowerTrackThreadFunc(ActiveNode* nodePtr)
{
	PowerTrackNode* node = dynamic_cast<PowerTrackNode*> (nodePtr);

	// Initial sleep time in order for enough data to be transmitted on the first message
	std::this_thread::sleep_for(std::chrono::milliseconds(node->POWER_TRACK_INITIAL_SLEEP));

	char buffer[1024];

	Timer timer;
	timer.start();
	while(true)
	{
		// Accept and recieve connections
		node->server.AcceptConnections();

		//Regulate the rate at whcih the messages are sent
		time.sleepUntil(node->m_LoopTime);

		MessagePtr powerTrack = std::make_unique<PowerStateMsg>(
			node->m_ArduinoPressure, node->m_ArduinoRudder, node->m_ArduinoSheet,
			node->m_ArduinoBattery, node->m_CurrentDataCurrent, node->m_CurrentDataVoltage,
			node->m_CurrentDataElement);

		node->m_MsgBus.sendMessage(std::move(powerTrack));

		int size = snprintf(buffer, 1024, "%d,%d,%d,%d,%f,%f,%d\n",
							(int)node->m_ArduinoPressure, (int)node->m_ArduinoRudder,
							(int)node->m_ArduinoSheet, (int)node->m_ArduinoBattery,
							(float)node->m_CurrentDataCurrent, (float)node->m_CurrentDataVoltage,
							(uint8_t)node->m_CurrentDataElement);
		if( size > 0 )
		{
			node->server.broadcast( (uint8_t*)buffer, size );
		}
		timer.reset();
	}
}