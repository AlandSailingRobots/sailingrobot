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
#include "Messages/ActuatorPositionMsg.h"


XbeeSyncNode* XbeeSyncNode::m_node = NULL;


XbeeSyncNode::XbeeSyncNode(MessageBus& msgBus, DBHandler& db) :
	ActiveNode(NodeID::xBeeSync, msgBus), m_xbee(false), m_initialised(false), m_db(db)
{
	m_firstMessageCall = true;
	m_sending = m_db.retrieveCellAsInt("xbee_config", "1", "send");
	m_receiving = m_db.retrieveCellAsInt("xbee_config", "1", "recieve");
	m_sendLogs = m_db.retrieveCellAsInt("xbee_config", "1", "send_logs");
	m_loopTime = stod(m_db.retrieveCell("xbee_config", "1", "loop_time"));
	m_pushOnlyLatestLogs = m_db.retrieveCellAsInt("xbee_config", "1", "push_only_latest_logs");
	msgBus.registerNode(*this, MessageType::VesselState);
	msgBus.registerNode(*this, MessageType::CourseData);
	msgBus.registerNode(*this, MessageType::WaypointData);
}

bool XbeeSyncNode::init()
{
	m_initialised = false;

	if(m_xbee.init("/dev/xbee", XBEE_BAUD_RATE))
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
        default:
            break;
    }

}

void XbeeSyncNode::sendMessage(const Message* msg)
{
	MessageSerialiser serialiser;
	msg->Serialise(serialiser);

	uint8_t* data = new uint8_t[serialiser.size()];
	memcpy(data, serialiser.data(), serialiser.size());

	m_xbee.transmit(data, serialiser.size());

	delete data;
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
			MessagePtr actuatorControl = std::make_unique<ActuatorPositionMsg>(ActuatorPositionMsg(deserialiser));
			m_node->m_MsgBus.sendMessage(std::move(actuatorControl));
		}
			break;
		default:
			break;
	}

	delete data;
	data = NULL;
}

void XbeeSyncNode::xBeeSyncThread(void* nodePtr)
{
	XbeeSyncNode* node = (XbeeSyncNode*)(nodePtr);

	node->m_xbee.setIncomingCallback(node->incomingMessage);

	while(true)
	{
		node->m_xbee.processRadioMessages();
	}
}
