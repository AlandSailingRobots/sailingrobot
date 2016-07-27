/****************************************************************************************
 *
 * File:
 * 		xBeeSyncNode.cpp
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

#include "xBeeSyncNode.h"
#include <chrono>
#include <thread>
#include <iostream>
#include <sstream>

//Should do most of the required things, see above
xBeeSyncNode::xBeeSyncNode(MessageBus& msgBus, DBHandler* db) :
	ActiveNode(NodeID::xBeeSync, msgBus), m_db(db)
{

}
bool xBeeSyncNode::init()
{

	m_sending = m_db->retrieveCellAsInt("xbee_config", "1", "send");
	m_receiving = m_db->retrieveCellAsInt("xbee_config", "1", "recieve");
	m_sendLogs = m_db->retrieveCellAsInt("xbee_config", "1", "send_logs");
	m_loopTime = stod(m_db->retrieveCell("xbee_config", "1", "loop_time"));
	m_pushOnlyLatestLogs = m_db->retrieveCellAsInt("xbee_config", "1", "push_only_latest_logs");

	bool rv = false;

	m_xbee_fd = m_xBee.init(); //Keep object in message architecture??

	if(m_xbee_fd < 0)
	{
		Logger::error("XbeeSync::%d Failed to initalise", __LINE__);
	}
	else
	{
		rv = true;
		Logger::info("Xbee initialised - receiving: %d sending: %d", m_receiving, m_sending);
		m_initialised = true;
	}

	return rv;
}

//DONE
void xBeeSyncNode::start(){

	if (m_initialised)
    {
		//not implemented in header
		m_messageTimeBuffer = m_loopTime;
        runThread(xBeeSyncThread);
    }
    else
    {
        Logger::error("%s Cannot start XBEESYNC thread as the node was not correctly initialised!", __PRETTY_FUNCTION__);
    }

}

//DONE
void xBeeSyncNode::processMessage(const Message* msgPtr)
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

//DONE
void xBeeSyncNode::sendVesselState(VesselStateMsg* msg){

	//Do not allow sending of vesselstate and logs at the same time; the output turns to mush
	if (!m_sendLogs && m_sending)
	{
		//The amount of seconds between expected calls from VesselStateMsg
		double messageStateInterval = 0.4;
		m_messageTimeBuffer -= messageStateInterval;

		double dummyAccelerationXYZ = -1; //Acceleration values
		int dummyArduinoValueN = -1; //Arduino values 0-3
		int dummyRudderState = -1;
		int dummySailState = -1;

		//Timestamp double->string conversion
		std::ostringstream strs;
		strs << msg->unixTime();
		std::string timeStampString = strs.str(); 

		//Make sure we do not send to often
		if (m_messageTimeBuffer <= 0){
			
			//Dummy values are used when not implemented in VesselStateMessage to fill gaps in expected xml string
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
				dummyArduinoValueN,
				dummyArduinoValueN,
				dummyArduinoValueN,
				dummyArduinoValueN,
				dummyRudderState,
				dummySailState
			);

			m_xBee.transmitData(m_xbee_fd,res_xml);

			//Reset after send, limit sending interval to node-wide loop time
			m_messageTimeBuffer = m_loopTime;
		}
	}

}

//DONE
void xBeeSyncNode::sendLogs(){
	
		if(m_sending && m_sendLogs) 
		{
			//Transmits latest/all logs over xbee network
			std::string logs = m_db->getLogs(m_pushOnlyLatestLogs);
			m_xBee.transmitData(m_xbee_fd, logs);
		}
}

//CO
void xBeeSyncNode::receiveControl(){
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

		//Send both at the same time

		if (rudder_cmd != -1 && sail_cmd != -1){
			Logger::info("Rudder command in xBeeSync::run = %d", rudder_cmd);
			Logger::info("Sail command in xBeeSync::run = %d", sail_cmd);
			ActuatorPositionMsg* actuatorControl = new ActuatorPositionMsg(rudder_cmd, sail_cmd);
			m_MsgBus.sendMessage(actuatorControl);

			//bool autorun = false;
			//Is external command still used?
			//m_external_command->setData(timestamp, autorun, rudder_cmd, sail_cmd);
		}

	}
}


//DONE
void xBeeSyncNode::xBeeSyncThread(void* nodePtr) {

	xBeeSyncNode* node = (xBeeSyncNode*)(nodePtr);

	while(true) {

		node->m_timer.reset();

		node->sendLogs();
		node->receiveControl();

		node->m_timer.sleepUntil(node->m_loopTime);
	}
}