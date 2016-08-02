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

	XbeeSyncNode::m_node = this;
	msgBus.registerNode(this, MessageType::VesselState);
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
			sendVesselState((VesselStateMsg*)msgPtr);
			break;
        default:
            break;
    }

}

void XbeeSyncNode::sendVesselState(VesselStateMsg* msg)
{
	MessageSerialiser serialiser;
	msg->Serialise(serialiser);

	uint8_t* data = new uint8_t[serialiser.size()];
	memcpy(data, serialiser.data(), serialiser.size());

	m_xbee.transmit(data, serialiser.size());

	delete data;


	//Do not allow sending of vesselstate and logs at the same time; double output turns into mush
	/*if (!m_sendLogs && m_sending)
	{
		bool enoughTimePassed = false;

		//check if enough time has passed since last call (determined by loop_time)
		if (!m_firstMessageCall)
		{
			if ((SysClock::unixTime() - m_lastMessageCallTime) >= m_loopTime){
				enoughTimePassed = true;
			}
		}
		else
		{
			m_firstMessageCall = false;
			enoughTimePassed = true;
		}
		m_lastMessageCallTime = SysClock::unixTime();

		//Dummy values are used when not implemented in VesselStateMessage to fill gaps in expected xml string
		double dummyAccelerationXYZ = -1; //Acceleration values
		int dummyRudderState = -1;
		int dummySailState = -1;

		//Timestamp double->string conversion
		std::ostringstream strs;
		strs << msg->unixTime();
		std::string timeStampString = strs.str(); 

		//Make sure we do not send too often
		if (enoughTimePassed){
			
			//If xml reading on the receiving end has been altered to work with other values it is safe to remove the dummy values.
			//If more variables have been implemented in VesselStateMessage it is safe to replace the dummy values.
			std::string res_xml = m_XML_log.log_xml(
				timeStampString,
				msg->windDir(),
				msg->windSpeed(),
				msg->compassHeading(),
				msg->compassPitch(),
				msg->compassRoll(),
				dummyAccelerationXYZ,
				dummyAccelerationXYZ,
				dummyAccelerationXYZ,
				msg->latitude(),
				msg->longitude(),
				msg->gpsHeading(),
				msg->speed(),
				msg->arduinoPressure(),
				msg->arduinoRudder(),
				msg->arduinoSheet(),
				msg->arduinoBattery(),
				dummyRudderState,
				dummySailState
			);

			m_xBee.transmitData(m_xbee_fd,res_xml);

		}
	}*/
}

/*
void XbeeSyncNode::sendLogs(){
	
		if(m_sending && m_sendLogs)
		{
			//Transmits latest/all logs over xbee network
			std::string logs = m_db.getLogs(m_pushOnlyLatestLogs);
			m_xBee.transmitData(m_xbee_fd, logs);
		}
}

void XbeeSyncNode::receiveControl(){
		if(m_receiving) {

		//Receive settings for rudder and sail by parsing xml message
		std::string res_xml = m_xBee.receiveXMLData(m_xbee_fd);
		
		//If return value equals -1, parsing failed...
		int rudder_cmd = m_XML_log.parse_rudCMD(res_xml);
		int sail_cmd = m_XML_log.parse_saiCMD(res_xml);
		std::string timestamp = m_XML_log.parse_time(res_xml);

		if(timestamp.length() > 0) {
			Logger::info("Timestamp in xBeeSync::run = %s", timestamp.c_str());
		}

		if (rudder_cmd != -1 && sail_cmd != -1){
			Logger::info("Rudder command in xBeeSync::run = %d", rudder_cmd);
			Logger::info("Sail command in xBeeSync::run = %d", sail_cmd);
			ActuatorPositionMsg* actuatorControl = new ActuatorPositionMsg(rudder_cmd, sail_cmd);
			m_MsgBus.sendMessage(actuatorControl);
			//PLANNED: Send externalCommandMsg here when implemented?
		}
	}
}*/

void XbeeSyncNode::incomingMessage(uint8_t* data, uint8_t size)
{
	MessageDeserialiser deserialiser(data, size);

	Message msg(deserialiser);
	deserialiser.resetInternalPtr();

	switch(msg.messageType())
	{
		case MessageType::ActuatorPosition:
		{
			ActuatorPositionMsg* actuatorControl = new ActuatorPositionMsg(deserialiser);
			m_node->m_MsgBus.sendMessage(actuatorControl);
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
