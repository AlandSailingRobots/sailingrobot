/****************************************************************************************
 *
 * File:
 * 		PowerTrackNode.cpp
 *
 * Purpose:
 *      Collects voltage and current data from the actuators, ECUs, and solar
 *      panel to track the power, sends it to the logger.
 *		
 *
 * Developer Notes:
 *		The ArduinoData may or may not be necessary.
 *
 ***************************************************************************************/

 #include "PowerTrackNode.h"

 #include <chrono>
 #include <thread>

 #include "Messages/PowerTrackMsg.h"
 #include "Math/CourseMath.h"
 #include "SystemServices/Logger.h"
 #include "SystemServices/Timer.h"
 #include "Database/DBHandler.h"

 #define POWER_TRACK_INITIAL_SLEEP 100

 PowerTrackNode::PowerTrackNode(MessageBus& msgBus, DBHandler& dbhandler, double loopTime)
    : ActiveNode(NodeID::PowerTrack, msgBus),
            m_CurrentSensorDataCurrent(0), m_CurrentSensorDataVoltage(0),  
            m_CurrentSensorDataElement((SensedElement)0),m_PowerBalance(0.f), m_Looptime(loopTime),
            m_lastElementRead(-1),m_db(dbhandler)
            
{
	msgBus.registerNode(*this, MessageType::CurrentSensorData);
}

//----------------------------------------------------------------------------------
PowerTrackNode::~PowerTrackNode(){}

//----------------------------------------------------------------------------------
bool PowerTrackNode::init()
{
	return true;
}

void PowerTrackNode::start()
{
	runThread(PowerTrackThreadFunc);
}

void PowerTrackNode::processMessage(const Message* msg)
{
	MessageType type = msg->messageType();

	switch(type)
	{

		case MessageType::CurrentSensorData:
			processCurrentSensorDataMessage((CurrentSensorDataMsg*) msg);
			break;

		default:
			return;
	}
}

void PowerTrackNode::processCurrentSensorDataMessage(CurrentSensorDataMsg* msg)
{
	m_CurrentSensorDataCurrent = msg->getCurrent();
	m_CurrentSensorDataVoltage = msg->getVoltage();
	m_CurrentSensorDataElement = msg->getSensedElement();
	m_Power = m_CurrentSensorDataVoltage * m_CurrentSensorDataCurrent;

	//if statement to ensure data flow is balanced
	// WARNING: this if statement only works if there are only 2 sensors, we want to switch between the readings
	//		    of the solar panel input and the battery output
	// ONE POSSIBLE FIX: move the update of m_lastElementRead in the SOLAR_PANEL and POWER_UNIT cases
	if ( m_CurrentSensorDataElement != m_lastElementRead ) {

		switch(m_CurrentSensorDataElement)
		{
			case SOLAR_PANEL :
				m_PowerBalance += m_Power;
				m_lastElementRead = m_CurrentSensorDataElement;
				break;

			case POWER_UNIT :
				m_PowerBalance -= m_Power;
				m_lastElementRead = m_CurrentSensorDataElement;
				break;

			default : 
				break;
		}

        //m_lastElementRead = m_CurrentSensorDataElement;
	}
}

void PowerTrackNode::PowerTrackThreadFunc(ActiveNode* nodePtr)
{
	PowerTrackNode* node = dynamic_cast<PowerTrackNode*> (nodePtr);

	// Initial sleep time in order for enough data to be transmitted on the first message
	std::this_thread::sleep_for(std::chrono::milliseconds(POWER_TRACK_INITIAL_SLEEP));

	Timer timer;
	timer.start();
	while(true)
	{
		//Regulate the rate at which the messages are sent
		timer.sleepUntil(node->m_Looptime);

        MessagePtr powerTrack = std::make_unique<PowerTrackMsg>(
            node->m_PowerBalance, node->m_CurrentSensorDataCurrent, node->m_CurrentSensorDataVoltage,
            node->m_CurrentSensorDataElement);

        node->m_MsgBus.sendMessage(std::move(powerTrack));

		// For testing only (to be removed soon/or shift to debug mode)
		Logger::debug("PowerTrackInfo: %f,%f,%f,%f,%d", (float)node->m_CurrentSensorDataCurrent,
			(float)node->m_CurrentSensorDataVoltage, (float)node->m_PowerBalance, (float)node->m_Power,
			(uint8_t)node->m_CurrentSensorDataElement);

		timer.reset();
	}
}
