/****************************************************************************************
 *
 * File:
 * 		XbeeSyncNode.cpp
 *
 * Purpose:
 *		Handles sending of logs or the current system state (xml-format) over xbee,
 * 		depending on the setting of send_logs. Also receives rudder and sail control messages
 *		over the xbee network.
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/


#include "XbeeSyncNode.h"
#include <cstring>

#include "Messages/ExternalControlMsg.h"
#include "Messages/ActuatorPositionMsg.h"

#include "SystemServices/Timer.h"


XbeeSyncNode* XbeeSyncNode::m_node = NULL;


XbeeSyncNode::XbeeSyncNode(MessageBus& msgBus, DBHandler& db) :
	ActiveNode(NodeID::xBeeSync, msgBus), m_initialised(false), m_db(db), m_dataLink("/dev/xbee", XBEE_BAUD_RATE), m_xbeeNetwork(m_dataLink, false)
{
	m_firstMessageCall = true;
	updateConfigsFromDB();
	msgBus.registerNode(*this, MessageType::VesselState);
	msgBus.registerNode(*this, MessageType::CourseData);
	msgBus.registerNode(*this, MessageType::WaypointData);
	msgBus.registerNode(*this, MessageType::ServerConfigsReceived);
	m_node = this;
}

bool XbeeSyncNode::init()
{
	m_initialised = false;

	if(m_dataLink.initialise(XBEE_PACKET_SIZE))
	{
		Logger::info("Xbee initialised - receiving: %d sending: %d", m_receiving, m_sending);
		m_initialised = true;
	}

	return m_initialised;
}

void XbeeSyncNode::start()
{
	if (m_initialised)
    {
        runThread(xBeeSyncThread);
    }
    else
    {
        Logger::error("%s Cannot start XBEESYNC thread as the node was not correctly initialised!", __PRETTY_FUNCTION__);
    }
}

void XbeeSyncNode::updateConfigsFromDB(){
	m_LoopTime = m_db.retrieveCellAsDouble("config_xbee","1","loop_time");
	m_receiving = m_db.retrieveCellAsInt("config_xbee", "1", "receive");
	m_sending = m_db.retrieveCellAsInt("config_xbee", "1", "send");
	m_sendLogs = m_db.retrieveCellAsInt("config_xbee", "1", "send_logs");
	m_pushOnlyLatestLogs = m_db.retrieveCellAsInt("config_xbee", "1", "push_only_latest_logs");
}

void XbeeSyncNode::processMessage(const Message* msgPtr)
{
    MessageType msgType = msgPtr->messageType();

    switch(msgType)
    {
		case MessageType::VesselState:
			sendMessage(msgPtr);
			break;
		case MessageType::CourseData:
			sendMessage(msgPtr);
			break;
		case MessageType::WaypointData:
			sendMessage(msgPtr);
			break;
		case MessageType::ServerConfigsReceived:
	        updateConfigsFromDB();
	        break;
        default:
            break;
    }

}

void XbeeSyncNode::sendMessage(const Message* msg)
{
	MessageSerialiser serialiser;
	msg->Serialise(serialiser);

	// The data is allocated on the stack, so is automatically cleaned up
	m_xbeeNetwork.transmit(serialiser.data(), serialiser.size());
}

void XbeeSyncNode::incomingMessage(uint8_t* data, uint8_t size)
{
	MessageDeserialiser deserialiser(data, size);

	Message msg(deserialiser);
	deserialiser.resetInternalPtr();

	switch(msg.messageType())
	{
		case MessageType::ActuatorPosition:
			{
				MessagePtr actuatorControl = std::make_unique<ActuatorPositionMsg>(deserialiser);
				m_node->m_MsgBus.sendMessage(std::move(actuatorControl));
				Logger::info("Actuator received");
			}
			break;
		case MessageType::ExternalControl:
			{
				MessagePtr externalControl = std::make_unique<ExternalControlMsg>(deserialiser);
				m_node->m_MsgBus.sendMessage(std::move(externalControl));
			}
			break;
		default:
			break;
	}

	delete data;
	data = NULL;
}

void XbeeSyncNode::xBeeSyncThread(ActiveNode* nodePtr)
{
	XbeeSyncNode* node = dynamic_cast<XbeeSyncNode*> (nodePtr);
	Timer timer;
	timer.start();

	node->m_xbeeNetwork.setIncomingCallback(node->incomingMessage);

	while(true)
	{
		node->m_xbeeNetwork.processRadioMessages();
		timer.sleepUntil(node->m_LoopTime);
		timer.reset();
	}
}
