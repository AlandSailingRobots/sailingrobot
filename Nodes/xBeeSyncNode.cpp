/****************************************************************************************
 *
 * File:
 * 		xBeeSyncNode.cpp
 *
 * Purpose:
 *		
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/


#include "xBeeSyncNode.h"
#include <chrono>
#include <thread>
#include <iostream>

// xBeeSync::xBeeSync(ExternalCommand* externalCommand, SystemState *systemState,
// 				   DBHandler* db, bool sendLogs, bool sending, bool receiving, double loopTime) :
// 	m_external_command(externalCommand),
// 	m_model(
// 		SystemStateModel(
// 			GPSModel("",PositionModel(0,0),0,0,0,0),
// 			WindsensorModel(0,0,0),
// 			CompassModel(0,0,0,AccelerationModel(0,0,0) ),
// 			AnalogArduinoModel(0, 0, 0, 0),
// 			0,
// 			0
// 		)
// 	),
// 	m_system_state(systemState),
// 	m_db(db),
// 	m_running(true), //*** RUNNING? INITIALIZED?
// 	m_sending(sending),
// 	m_receiving(receiving),
// 	m_sendLogs(sendLogs),
// 	m_loopTime(loopTime)
// { }

//Should do most of the required things, see above
xBeeSyncNode::xBeeSyncNode(MessageBus& msgBus, DBHandler* db, bool sendLogs, bool sending, bool receiving, double loopTime)
        :ActiveNode(NodeID::xBeeSync, msgBus), m_db(db), m_sending(sending), m_receiving(receiving), m_sendLogs(sendLogs), m_loopTime(loopTime)
        {

        }

//DONE?
bool xBeeSyncNode::init()
{

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
void xBeeSync::start(){

	if (m_initialised)
    {
		//not implemented
		m_messageTimeBuffer = m_loopTime;
        runThread(xBeeSyncThread);
    }
    else
    {
        Logger::error("%s Cannot start HTTPSYNC thread as the node was not correctly initialised!", __PRETTY_FUNCTION__);
    }

}

//CO
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

//CO
void xBeeSyncNode::sendVesselState(VesselStateMsg* msg){

	//Do not allow sending of vesselstate and logs at the same time; the output turns to mush
	if (!m_sendLogs && m_sending)
	{
		//The amount of seconds between expected calls from VesselStateMsg
		double messageStateInterval = 0.4;
		m_messageTimeBuffer -= messageStateInterval;

		//Make sure we do not send to often
		if (m_messageTimeBuffer <= 0){
			
			//TODO: ALL CAPS ARE DUMMIES, PUT IN MOCKS OR GET REAL VALUES
			std::string res_xml = m_XML_log.log_xml(
				msg->unixTime(),
				msg->windDir(),
				msg->windSpeed(),
				msg->compassHeading(),
				msg->compassPitch(),
				msg->compassRoll(),
				ACCELERATIONX,
				ACCELERATIONY,
				ACCELERATIONZ,
				msg->latitude(),
				msg->longitude(),
				msg->gpsHeading(),
				msg->speed(),
				ARDUINOVALUE0,
				ARDUINOVALUE1,
				ARDUINOVALUE2,
				ARDUINOVALUE3,
				RUDDERSTATE,
				SAILSTATE
			);

			m_xBee.transmitData(m_xbee_fd,res_xml);

			//Reset after send
			m_messageTimeBuffer = m_loopTime;
		}
	}

}

//CO
void xBeeSyncNode::sendLogs(){
	
		if(m_sending && m_sendLogs) 
		{
			//Transmits latest logs over xbee network
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

			bool autorun = false;
			//Is external command still used?
			m_external_command->setData(timestamp, autorun, rudder_cmd, sail_cmd);
		}

	}
}

//Not done: Mostly copied from xbeeSync.cpp - implement when a replacement exists for system state model
//TODO: Send back message
void xBeeSyncNode::xBeeSyncThread(void nodePtr) {

	xBeeSyncNode* node = (xBeeSyncNode*)(nodePtr);

	m_pushOnlyLatestLogs = m_db->retrieveCellAsInt("xbee_config", "1", "push_only_latest_logs");
	Logger::info("*xBeeSync thread started.");

	while(true) {

		m_timer.reset();

		//Stores current system state in local model (Outdated)
		//m_system_state->getData(m_model);

		node->SendLogs();
		node->receiveControl();

		m_timer.sleepUntil(m_loopTime);
	}
	Logger::info("*xBeeSync thread exited.");
}