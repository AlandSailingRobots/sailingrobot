
#pragma once

#include "ActiveNode.h"
#include "dbhandler/DBHandler.h"
#include "logger/Logger.h"
#include "models/SystemStateModel.h"


#include <chrono>
#include <thread>
#include <curl/curl.h>
#include <string>


class HTTPSyncNode : public ActiveNode{


	public:

		HTTPSyncNode(MessageBus& msgBus,DBHandler *db,int delay, bool removeLogs);
        
        bool init();
        void start();
        void processMessage(const Message* message);


	private:

        bool m_initialised;

		std::string m_shipID;
		std::string m_shipPWD;
		std::string m_serverURL;

		CURL *curl;
		CURLcode m_res;
		bool m_reportedConnectError;

		bool m_removeLogs;
		int m_delay;
		int m_pushOnlyLatestLogs;

		DBHandler *m_dbHandler;

		std::string getData(std::string call);

		bool performCURLCall(std::string data, std::string call, std::string& response);
        void getWaypointsFromServer();
        void getConfigsFromServer();
        bool checkIfNewConfigs();
		bool checkIfNewWaypoints();

        void pushDatalogs();
		void pushWaypoints();
		void pushConfigs();

        static void HTTPSyncThread(void* nodePtr);
};

