
#pragma once

#include "../Database/DBHandler.h"
#include "../MessageBus/ActiveNode.h"
#include "../SystemServices/Logger.h"

#include <curl/curl.h>
#include <atomic>
#include <chrono>
#include <string>
#include <thread>

class HTTPSyncNode : public ActiveNode {
   public:
    HTTPSyncNode(MessageBus& msgBus, DBHandler* dbhandler);

    virtual ~HTTPSyncNode() {}

    ///----------------------------------------------------------------------------------
    /// Retrieves server settings from database and initialises curl
    ///
    ///----------------------------------------------------------------------------------
    bool init();
    void start();
    void stop();

    ///----------------------------------------------------------------------------------
    /// Pushes waypoints or configurations on new local changes
    /// (Example of cause: xbeeSync functions)
    ///----------------------------------------------------------------------------------
    void processMessage(const Message* message);
    ///----------------------------------------------------------------------------------
    /// Push functions: sends local data to server using curl
    ///----------------------------------------------------------------------------------
    bool pushDatalogs();
    bool pushWaypoints();
    bool pushConfigs();
    ///----------------------------------------------------------------------------------
    /// Updates local waypoints using new server data if any
    ///----------------------------------------------------------------------------------
    bool getWaypointsFromServer();

    ///----------------------------------------------------------------------------------
    /// Same as above but for configuration data
    ///----------------------------------------------------------------------------------
    bool getConfigsFromServer();

   private:
    ///----------------------------------------------------------------------------------
    /// Sends server request in curl format - used for all syncing functionality
    ///----------------------------------------------------------------------------------
    bool performCURLCall(std::string data, std::string call, std::string& response);

    bool checkIfNewConfigs();
    bool checkIfNewWaypoints();

    ///----------------------------------------------------------------------------------
    /// Node thread: Calls all syncing functions while running
    ///----------------------------------------------------------------------------------
    static void HTTPSyncThread(ActiveNode* nodePtr);

    void updateConfigsFromDB();

    ///----------------------------------------------------------------------------------
    /// Convenience function: creates curl call from argument and returns response (json data)
    ///----------------------------------------------------------------------------------
    std::string getData(std::string call);

    bool m_initialised;

    std::string m_shipID;
    std::string m_shipPWD;
    std::string m_serverURL;

    CURL* curl;
    CURLcode m_res;
    bool m_reportedConnectError;

    ///----------------------------------------------------------------------------------
    /// Determines whether or not to clear all local logs after a successful push to server
    ///----------------------------------------------------------------------------------
    bool m_removeLogs;
    double m_LoopTime;  // units : seconds (ex : 0.5 s)
    int m_pushOnlyLatestLogs;

    std::atomic<bool> m_Running;
    DBHandler* m_dbHandler;
};
