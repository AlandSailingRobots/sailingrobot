
/****************************************************************************************
 *
 * File:
 * 		HTTPSyncNode.cpp
 *
 * Purpose:
 *		Handles retrieval and sending of logs, waypoints and configs between database and server.
 *      Also notifies messagebus when new serverdata arrives.
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/

#include "HTTPSyncNode.h"
#include "Messages/LocalConfigChangeMsg.h"
#include "Messages/LocalWaypointChangeMsg.h"
#include "Messages/ServerConfigsReceivedMsg.h"
#include "Messages/ServerWaypointsReceivedMsg.h"
#include "SystemServices/Timer.h"

#include <atomic>


size_t write_to_string(void *ptr, size_t size, size_t count, void *stream) {
    ((std::string*)stream)->append((char*)ptr, 0, size*count);
    return size*count;
}

HTTPSyncNode::HTTPSyncNode(MessageBus& msgBus, DBHandler *dbhandler, int delay, bool removeLogs)
	:ActiveNode(NodeID::HTTPSync, msgBus), m_removeLogs(removeLogs), m_delay(delay), m_dbHandler(dbhandler)
{
    msgBus.registerNode( *this, MessageType::LocalWaypointChange);
    msgBus.registerNode( *this, MessageType::LocalConfigChange);
    msgBus.registerNode( *this, MessageType::ServerConfigsReceived);
}

bool HTTPSyncNode::init()
{


    m_initialised = false;

    m_reportedConnectError = false;

    m_pushOnlyLatestLogs = m_dbHandler->retrieveCellAsInt("config_httpsync", "1", "push_only_latest_logs");

    m_shipID = m_dbHandler->retrieveCell("config_HTTPSyncNode", "1", "boat_id");
    m_serverURL = m_dbHandler->retrieveCell("config_HTTPSyncNode", "1", "srv_addr");
    m_shipPWD = m_dbHandler->retrieveCell("config_HTTPSyncNode", "1", "boat_pwd");

    m_initialised = true;

    return m_initialised;

}

void HTTPSyncNode::start(){

    if (m_initialised)
    {
        m_Running.store(true);
        runThread(HTTPSyncThread);
    }
    else
    {
        Logger::error("%s Cannot start HTTPSYNC thread as the node was not correctly initialised!", __PRETTY_FUNCTION__);
    }
}

void HTTPSyncNode::stop()
{
    m_Running.store(false);
}

void HTTPSyncNode::updateConfigsFromDB()
{
    m_removeLogs = m_dbHandler->retrieveCellAsInt("httpsync_config","1","remove_logs");
    m_delay = m_dbHandler->retrieveCellAsInt("httpsync_config","1","delay");
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
        case MessageType::ServerConfigsReceived:
            updateConfigsFromDB();
            break;
        default:
            break;
        //Potentially: on new log data. Currently sending constantly in thread
    }

}

void HTTPSyncNode::HTTPSyncThread(ActiveNode* nodePtr){

    HTTPSyncNode* node = dynamic_cast<HTTPSyncNode*> (nodePtr);


    Logger::info("HTTPSync thread has started");

    curl_global_init(CURL_GLOBAL_ALL);

    node->pushWaypoints();
    node->pushConfigs();

    Timer timer;
  	timer.start();
    while(node->m_Running.load() == true)
    {
        std::cout << "\n /* Httpsync loop actif */ \n";
        node->getConfigsFromServer();
        node->getWaypointsFromServer();
        node->pushDatalogs();

        timer.sleepUntil(node->m_delay);
        timer.reset();

    }
    std::cout << "/* Out loop */" << '\n';
    curl_global_cleanup();
    std::cout << "/* Curl globql cleanup */" << '\n';
    Logger::info("HTTPSync thread has exited");
    std::cout << "/* Exit httpsync */" << '\n';
}

bool HTTPSyncNode::pushDatalogs() {
    std::string response = "";
    if(performCURLCall(m_dbHandler->getLogs(m_pushOnlyLatestLogs), "pushAllLogs", response))
    {
         //remove logs after push
        if(m_removeLogs) {
            m_dbHandler->clearLogs();
        }
        return true;
    }
    else if(!m_reportedConnectError)
    {
        Logger::warning("%s Could not push logs to server:", __PRETTY_FUNCTION__);
    }

    return false;
}

bool HTTPSyncNode::pushWaypoints()
{
	std::string waypointsData = m_dbHandler->getWaypoints();
	if (waypointsData.size() > 0)
	{
		std::string response;
		if(performCURLCall(waypointsData,"pushWaypoints", response))
		{
            Logger::info("Waypoints pushed to server");
            return true;
		}
		else if(!m_reportedConnectError)
		{
			Logger::warning("%s Failed to push waypoints to server", __PRETTY_FUNCTION__);
		}
	}
    return false;
}

bool HTTPSyncNode::pushConfigs() {
    std::string response;

	if(performCURLCall(m_dbHandler->getConfigs(), "pushConfigs", response))
	{
		Logger::info("Configs pushed to server");
        return true;
	}
	else if(!m_reportedConnectError)
	{
		Logger::warning("%s Error: ", __PRETTY_FUNCTION__);
	}

    return false;
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


bool HTTPSyncNode::getConfigsFromServer() {

    if(checkIfNewConfigs())
    {
        std::string configs = getData("getAllConfigs");
        if (configs.size() > 0)
        {
            m_dbHandler->updateConfigs(configs);
            if (not m_dbHandler->updateTable("state", "configs_updated", "1", "1"))
            {
                Logger::error("%s Error updating state table",__PRETTY_FUNCTION__);
                return false;
            }

            MessagePtr newServerConfigs = std::make_unique<ServerConfigsReceivedMsg>();
            m_MsgBus.sendMessage(std::move(newServerConfigs));
            Logger::info("Configuration retrieved from remote server");
            return true;
        }
        else if(!m_reportedConnectError)
        {
            Logger::error("%s Error: %s", __PRETTY_FUNCTION__);
        }

    }
    return false;
}

bool HTTPSyncNode::getWaypointsFromServer() {

    if(checkIfNewWaypoints()){

        std::string waypoints = getData("getWaypoints");
        if (waypoints.size() > 0)
        {
            if (m_dbHandler->updateWaypoints(waypoints))
            {
                //EVENT MESSAGE - REPLACES OLD CALLBACK, CLEAN OUT CALLBACK REMNANTS IN OTHER CLASSES
                MessagePtr newServerWaypoints = std::make_unique<ServerWaypointsReceivedMsg>();
                m_MsgBus.sendMessage(std::move(newServerWaypoints));

                Logger::info("Waypoints retrieved from remote server");
                return true;
            }

        }
        else if(!m_reportedConnectError)
        {
            Logger::warning("%s Could not fetch any new waypoints",__PRETTY_FUNCTION__);
        }

    }
    return false;
}

bool HTTPSyncNode::performCURLCall(std::string data, std::string call, std::string& response) {
    std::string serverCall = "";

    if(data != "")
        serverCall = "serv="+call + "&id="+m_shipID+"&pwd="+m_shipPWD+"&data="+data;
    else
        serverCall = "serv="+call + "&id="+m_shipID+"&pwd="+m_shipPWD;
        //example: serv=getAllConfigs&id=BOATID&pwd=BOATPW

    curl = curl_easy_init();
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
                curl_easy_cleanup(curl);
				return false;
			}
			else if(m_res == CURLE_COULDNT_CONNECT)
			{
				m_reportedConnectError = true;
			}
            curl_easy_cleanup(curl);
			return false;
		}
		else
		{
			m_reportedConnectError = false;
		}
        curl_easy_cleanup(curl);
    }else{
        fprintf(stderr, "CURL IS FALSE");
    }

    return true;
}
