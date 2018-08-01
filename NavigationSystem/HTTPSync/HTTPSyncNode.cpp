
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
#include "../Messages/LocalConfigChangeMsg.h"
#include "../Messages/LocalWaypointChangeMsg.h"
#include "../Messages/ServerConfigsReceivedMsg.h"
#include "../Messages/ServerWaypointsReceivedMsg.h"
#include "../SystemServices/Timer.h"
#include "../SystemServices/Wrapper.h"

#include <atomic>

// The callbacks CANNOT be non-static class member functions
static size_t write_to_string(void* ptr, size_t size, size_t count, void* stream) {
    ((std::string*)stream)->append((char*)ptr, 0, size * count);
    return size * count;
}

HTTPSyncNode::HTTPSyncNode(MessageBus& msgBus, DBHandler* dbhandler)
    : ActiveNode(NodeID::HTTPSync, msgBus),
      m_removeLogs(1),
      m_LoopTime(0.5),
      m_dbHandler(dbhandler) {
    msgBus.registerNode(*this, MessageType::LocalWaypointChange);
    msgBus.registerNode(*this, MessageType::WaypointData);
    msgBus.registerNode(*this, MessageType::LocalConfigChange);
    msgBus.registerNode(*this, MessageType::ServerConfigsReceived);
}

bool HTTPSyncNode::init() {
    m_initialised = false;
    m_reportedConnectError = false;

    // TODO this should be a single query
    m_dbHandler->getConfigFrom(m_serverURL, "srv_addr", "config_httpsync");
    m_dbHandler->getConfigFrom(m_shipID, "boat_id", "config_httpsync");
    m_dbHandler->getConfigFrom(m_shipPWD, "boat_pwd", "config_httpsync");
    updateConfigsFromDB();

    m_dataLogsSystemLastId = 0;
    m_connectionErrors = 0;
    m_sendFails = 0;

    m_initialised = true;

    return m_initialised;
}

void HTTPSyncNode::start() {
    if (m_initialised) {
        m_Running.store(true);
        runThread(HTTPSyncThread);
    } else {
        Logger::error("%s Cannot start HTTPSYNC thread as the node was not correctly initialised!",
                      __PRETTY_FUNCTION__);
    }
}

void HTTPSyncNode::stop() {
    m_Running.store(false);
}

void HTTPSyncNode::updateConfigsFromDB() {
    m_dbHandler->getConfigFrom(m_removeLogs, "remove_logs", "config_httpsync");
    m_dbHandler->getConfigFrom(m_pushOnlyLatestLogs, "push_only_latest_logs", "config_httpsync");
    m_dbHandler->getConfigFrom(m_LoopTime, "loop_time", "config_httpsync");
}

void HTTPSyncNode::processMessage(const Message* msgPtr) {
    MessageType msgType = msgPtr->messageType();

    switch (msgType) {
        case MessageType::LocalWaypointChange:
            pushWaypoints();
            break;
        case MessageType::WaypointData:
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
            // Potentially: on new log data. Currently sending constantly in thread
    }
}

void HTTPSyncNode::HTTPSyncThread(ActiveNode* nodePtr) {
    HTTPSyncNode* node = dynamic_cast<HTTPSyncNode*>(nodePtr);

    Logger::info("HTTPSync thread has started");

    curl_global_init(CURL_GLOBAL_ALL);

    node->pushWaypoints();
    node->pushConfigs();

    Timer timer;
    timer.start();

    while (node->m_Running.load() == true) {
        node->getConfigsFromServer();
        node->getWaypointsFromServer();
        node->pushDatalogs();

        timer.sleepUntil(node->m_LoopTime + (node->m_connectionErrors > 60 ? 60 : node->m_connectionErrors));
        timer.reset();
    }
    curl_global_cleanup();
    Logger::info("HTTPSync thread has exited");
}

bool HTTPSyncNode::pushDatalogs() {
    std::string response;
    unsigned int latestId = m_dbHandler->getTableId("dataLogs_system");

    if (m_pushOnlyLatestLogs) {
        m_dataLogsSystemLastId = std::max(m_dataLogsSystemLastId, latestId - 1);
    }
    if (!m_dataLogsSystemLastId) {
        m_dataLogsSystemLastId = latestId;
        Logger::warning(
            "%s() Last pushed log index unavailable, will send new log items from here on (items "
            ">%d)",
            __FUNCTION__, latestId);
    }

    unsigned int pushedId = m_dataLogsSystemLastId;

    if (pushedId < latestId) {
        unsigned int chunkEnd = std::min(pushedId + (m_connectionErrors ? 1 : 5), latestId);
        if (m_connectionErrors) {
            Logger::info("Trying to push single log item %d to the server", chunkEnd);
        } else {
            Logger::info("Trying to push %d log items %d-%d to the server", chunkEnd - pushedId, pushedId + 1, chunkEnd);
        }
        std::string logs = m_dbHandler->getLogsAsJSON(pushedId, chunkEnd);

        if (logs.empty()) {
            Logger::warning("%s Not pushing empty logs to server",
                            __PRETTY_FUNCTION__);  // disable again when debugged
            return false;
        }

        // TODO: Behave differently on connection error or on service errors from server
        auto sendBegin = std::chrono::high_resolution_clock::now();
        if (performCURLCall(logs, "pushAllLogs", response)) {
            auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                          std::chrono::high_resolution_clock::now() - sendBegin)
                          .count();
            int items = pushedId - m_dataLogsSystemLastId;
            Logger::info("Pushed %d log items %d-%d to the server in %d ms (avg. %.02f ms/item)",
                         items, m_dataLogsSystemLastId, pushedId, ms,
                         (float)ms / (float)items);
            m_dataLogsSystemLastId = pushedId;
        } else {
            Logger::warning("%s Failed pushing logs to server, will retry later",
                            __PRETTY_FUNCTION__);
            return false;
        }
    }
    return true;
}

bool HTTPSyncNode::pushWaypoints() {
    std::string response;
    std::string waypointsData = m_dbHandler->getWayPointsAsJSON();
    if (waypointsData.size() > 0) {
        if (performCURLCall(waypointsData, "pushWaypoints", response)) {
            Logger::info("Waypoints pushed to the server");
            return true;
        }
    }
    Logger::error("%s Failed to push way points table to the server", __PRETTY_FUNCTION__);
    return false;
}

bool HTTPSyncNode::pushConfigs() {
    std::string response;

    if (performCURLCall(m_dbHandler->getConfigs(), "pushConfigs", response)) {
        Logger::info("Configs pushed to the server");
        return true;
    }
    Logger::error("%s Failed to push configs to the server", __PRETTY_FUNCTION__);
    return false;
}

std::string HTTPSyncNode::getData(std::string call) {
    std::string response = "";

    if (performCURLCall("", call, response)) {
        return response;
    } else {
        return "";
    }
}

bool HTTPSyncNode::checkIfNewConfigs() {
    std::string result = getData("checkIfNewConfigs");

    if (result.length()) {
        JSON js = JSON::parse(result);
        if (safe_stoi(js["configs_updated"])) {
            return true;
        } else {
            return false;
        }
    }
    Logger::error("%s Could not check for new configs on the server!", __PRETTY_FUNCTION__);
    return false;
}

bool HTTPSyncNode::checkIfNewWaypoints() {
    std::string result = getData("checkIfNewWaypoints");

    if (result.length()) {
        JSON js = JSON::parse(result);
        if (safe_stoi(js["route_updated"])) {
            return true;
        } else {
            return false;
        }
    }
    Logger::error("%s Could not check for new waypoints on the server!", __PRETTY_FUNCTION__);
    return false;
}

bool HTTPSyncNode::getConfigsFromServer() {
    if (checkIfNewConfigs()) {
        std::string configs = getData("getAllConfigs");
        if (!configs.empty()) {
            m_dbHandler->receiveConfigs(configs);
            /*
                        This does not work as there no longer seems to exist any "state" table /Kåre
                        if (not m_dbHandler->updateTableColumnIdValue("state", "configs_updated", 1,
               1)) { Logger::error("%s Error updating state table", __PRETTY_FUNCTION__); return
               false;
                        }
            */
            MessagePtr newServerConfigs = std::make_unique<ServerConfigsReceivedMsg>();
            m_MsgBus.sendMessage(std::move(newServerConfigs));
            Logger::info("Configuration retrieved from remote server");
            return true;
        }
        Logger::error("%s Could not retrieve configuration from the server!", __PRETTY_FUNCTION__);
    }
    return false;
}

bool HTTPSyncNode::getWaypointsFromServer() {
    if (checkIfNewWaypoints()) {
        std::string waypoints = getData("getWaypoints");
        if (!waypoints.empty()) {
            if (m_dbHandler->receiveWayPoints(waypoints)) {
                MessagePtr newServerWaypoints = std::make_unique<ServerWaypointsReceivedMsg>();
                m_MsgBus.sendMessage(std::move(newServerWaypoints));
                Logger::info("Waypoints retrieved from remote the server");
                return true;
            }
        }
        Logger::error("%s Could not retrieve new waypoints from the server!", __PRETTY_FUNCTION__);
    }
    return false;
}

bool HTTPSyncNode::performCURLCall(std::string data, std::string call, std::string& response) {
    std::string serverCall = "";

    // std::cout << "/* Request : " << call << " */" << '\n';
    serverCall = "serv=" + call + "&id=" + m_shipID + "&pwd=" + m_shipPWD;
    if (data.length()) {
        serverCall += "&data=" + data;
    }
    // example: serv=getAllConfigs&id=BOATID&pwd=BOATPW
    // std::cout << "/* Server call : " << serverCall.substr(0, 150) << " */" << '\n';

    std::lock_guard<std::mutex> lock_guard(m_lock);
    curl = curl_easy_init();
    if (curl) {
        // https://curl.haxx.se/libcurl/c/threadsafe.html
        curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1L);

        // Fail on errors
        curl_easy_setopt(curl, CURLOPT_FAILONERROR, 1L);

        // Human readable error message buffer
        char errbuf[CURL_ERROR_SIZE];
        curl_easy_setopt(curl, CURLOPT_ERRORBUFFER, errbuf);
        errbuf[0] = 0;  // empty before request

        // Send data through curl with POST
        curl_easy_setopt(curl, CURLOPT_URL, m_serverURL.c_str());
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, serverCall.c_str());
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_to_string);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);

        // Perform the request, m_res will get the return code
        m_res = curl_easy_perform(curl);
        curl_easy_cleanup(curl);  // This gave double-free SIGABRT

        // Check for errors
        // std::cout << "/* Reponse serveur : " << response << "\n*/" << "\n\n\n\n";
        if (m_res == CURLE_OK) {
            if (m_reportedConnectError) {
                m_reportedConnectError = false;
                Logger::info("Re-established server connection after %d failed attempts",
                             m_connectionErrors);
                m_connectionErrors = 0;
            }
            return true;  // All is well
        } else {
            // if (!m_reportedConnectError) {
            if (strlen(errbuf)) {
                Logger::error("%s() %s (code %d, errcnt %d)", __FUNCTION__, errbuf, m_res,
                              m_connectionErrors);
            } else {
                Logger::error("%s() %s (code %d, errcnt %d)", __FUNCTION__,
                              curl_easy_strerror(m_res), m_res, m_connectionErrors);
            }
            m_reportedConnectError = true;
            //}
        }
    } else {
        // fprintf(stderr, "CURL IS FALSE");
        Logger::error("%s ERROR: problems with curllib, curl is false!", __PRETTY_FUNCTION__);
    }
    m_connectionErrors++;
    return false;
}
