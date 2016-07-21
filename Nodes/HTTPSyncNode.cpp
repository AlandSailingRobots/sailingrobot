#include "HTTPSyncNode.h"
#include "Messages/LocalConfigChangeMsg.h"
#include "Messages/LocalWaypointChangeMsg.h"
#include "Messages/ServerConfigsReceivedMsg.h"
#include "Messages/ServerWaypointsReceivedMsg.h"

/*

    PURPOSE:
        HANDLE RETRIEVAL AND WRITING TO DATABASE AUTONOMOUSLY UPON LOCAL OR SERVERSIDE CHANGES
        PUSH LOGS ON AN INTERVAL
        NOTIFY MESSAGEBUS WHEN NEW SERVERDATA ARRIVES

*/

size_t write_to_string(void *ptr, size_t size, size_t count, void *stream) {
    ((std::string*)stream)->append((char*)ptr, 0, size*count);
    return size*count;
}

HTTPSyncNode::HTTPSyncNode(MessageBus& msgBus, DBHandler *db, int delay, bool removeLogs)
	:ActiveNode(NodeID::HTTPSync, msgBus), m_removeLogs(removeLogs), m_delay(delay), m_dbHandler(db)
{
    
}

bool HTTPSyncNode::init()
{

    m_initialised = false;

    curl = curl_easy_init();
    m_reportedConnectError = false;

    m_pushOnlyLatestLogs = m_dbHandler->retrieveCellAsInt("httpsync_config", "1", "push_only_latest_logs");

    setShipID( m_dbHandler->retrieveCell("server", "1", "boat_id") );
    setShipPWD( m_dbHandler->retrieveCell("server", "1", "boat_pwd") );
    setServerURL( m_dbHandler->retrieveCell("server", "1", "srv_addr") );

    m_initialised = true;
    Logger::info("HTTPSyncNode init() successful");

    return m_initialised;

}

void HTTPSyncNode::start(){

    if (m_initialised)
    {
        runThread(HTTPSyncThread);
    }
    else
    {
        Logger::error("%s Cannot start HTTPSYNC thread as the node was not correctly initialised!", __PRETTY_FUNCTION__);
    }

}

void HTTPSyncNode::processMessage(const Message* msgPtr)
{
    MessageType msgType = msgPtr->messageType();

    switch(msgType)
    {
        case MessageType::LocalWaypointChange:
            pushWaypoints();
            break;
        case MessageType::LocalConfigChange:
            pushConfigs();
            break;
        default:
            break;
        //Potentially: on new log data. Currently sending constantly in thread
    }

}

void HTTPSyncNode::HTTPSyncThread(void* nodePtr){

    HTTPSyncNode* node = (HTTPSyncNode*)(nodePtr);

    Logger::info("HTTPSync thread has started");

    while(true)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(node->m_delay));

        node->getConfigsFromServer();
        node->getWaypointsFromServer();
        node->pushDatalogs();
    }

    Logger::info("HTTPSync thread has exited");
}

void HTTPSyncNode::pushDatalogs() {
    std::string response = "";
    if(performCURLCall(m_dbHandler->getLogs(m_pushOnlyLatestLogs), "pushAllLogs", response))
    {
         //remove logs after push
        if(m_removeLogs) {
            m_dbHandler->clearLogs();
        }

        Logger::info("Logs pushed to server");
    }
    else if(!m_reportedConnectError)
    {
        Logger::warning("%s Could not push logs to server:", __PRETTY_FUNCTION__);
    }
}

void HTTPSyncNode::pushWaypoints() 
{
	std::string waypointsData = m_dbHandler->getWaypoints();
	if (waypointsData.size() > 0)
	{
		std::string response;
		if(performCURLCall(waypointsData,"pushWaypoints", response))
		{
			Logger::info("Waypoints pushed to server");
		}
		else if(!m_reportedConnectError)
		{
			Logger::warning("%s Failed to push waypoints to server", __PRETTY_FUNCTION__);
		}
	}
}

void HTTPSyncNode::pushConfigs() {
    std::string response;

	if(performCURLCall(m_dbHandler->getConfigs(), "pushConfigs", response))
	{
		Logger::info("Configs pushed to server");
	}
	else if(!m_reportedConnectError)
	{
		Logger::warning("%s Error: ", __PRETTY_FUNCTION__);
	}
}


void HTTPSyncNode::setShipID(std::string ID) {
    m_shipID = ID;
}

void HTTPSyncNode::setShipPWD(std::string PWD) {
    m_shipPWD = PWD;
}

void HTTPSyncNode::setServerURL(std::string URL) {
    m_serverURL = URL;
}

std::string HTTPSyncNode::getData(std::string call) {
	std::string response;
    if(performCURLCall("",call, response))
    {
    	return response;
    }
    else
    {
    	return "";
    }
}

bool HTTPSyncNode::checkIfNewConfigs() {
    if (getData("checkIfNewConfigs") == "1")
        return true;

    return false;
}

bool HTTPSyncNode::checkIfNewWaypoints(){
    if (getData("checkIfNewWaypoints") == "1")
   	    return true;

    return false;
}


void HTTPSyncNode::getConfigsFromServer() {

    if(checkIfNewConfigs())
    {
        std::string configs = getData("getAllConfigs");
        if (configs.size() > 0)
        {
            m_dbHandler->updateConfigs(configs);
            if (not m_dbHandler->updateTable("state", "configs_updated", "1", "1"))
            {
                Logger::error("%s Error updating state table",__PRETTY_FUNCTION__);
                return;
            }

            ServerConfigsReceivedMsg* newServerConfigs = new ServerConfigsReceivedMsg();
            m_MsgBus.sendMessage(newServerConfigs);
            Logger::info("Configuration retrieved from remote server");
        }
        else if(!m_reportedConnectError)
        {
            Logger::error("%s Error: %s", __PRETTY_FUNCTION__);
        }
    }
}

void HTTPSyncNode::getWaypointsFromServer() {

    if(checkIfNewWaypoints()){

        std::string waypoints = getData("getWaypoints");
        if (waypoints.size() > 0)
        {
            if (m_dbHandler->updateWaypoints(waypoints))
            {
                //EVENT MESSAGE - REPLACES OLD CALLBACK, CLEAN OUT CALLBACK REMNANTS IN OTHER CLASSES
                ServerWaypointsReceivedMsg* newServerWaypoints = new ServerWaypointsReceivedMsg();
                m_MsgBus.sendMessage(newServerWaypoints);

                Logger::info("Waypoints retrieved from remote server");
            }
        }
        else if(!m_reportedConnectError)
        {
            Logger::warning("%s Could not fetch any new waypoints",__PRETTY_FUNCTION__);
        }
    }
}

bool HTTPSyncNode::performCURLCall(std::string data, std::string call, std::string& response) {
    std::string serverCall = "";

    if(data != "")
        serverCall = "serv="+call + "&id="+m_shipID+"&pwd="+m_shipPWD+"&data="+data;
    else
        serverCall = "serv="+call + "&id="+m_shipID+"&pwd="+m_shipPWD;
        //example: serv=getAllConfigs&id=BOATID&pwd=BOATPW

    if(curl) {
    	//Send data through curl with POST
		curl_easy_setopt(curl, CURLOPT_URL, m_serverURL.c_str());
		curl_easy_setopt(curl, CURLOPT_POSTFIELDS, serverCall.c_str());
		curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_to_string);
		curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);
		/* Perform the request, res will get the return code */
		m_res = curl_easy_perform(curl);
		/* Check for errors */
		if (m_res != CURLE_OK)
		{
			if(!m_reportedConnectError)
			{
				Logger::error("%s Error: %s", __PRETTY_FUNCTION__, curl_easy_strerror(m_res));
			}
			if(m_res == CURLE_COULDNT_CONNECT && m_reportedConnectError)
			{
				return false;
			}
			else if(m_res == CURLE_COULDNT_CONNECT)
			{
				m_reportedConnectError = true;
			}
			return false;
		}
		else
		{
			m_reportedConnectError = false;
		}
    }

    return true;
}
