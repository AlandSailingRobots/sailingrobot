#ifndef __HTTPSYNCNODE_H__
#define __HTTPSYNCNODE_H__

#include "../dbhandler/DBHandler.h"
#include "logger/Logger.h"
#include "models/SystemStateModel.h"

#include <chrono>
#include <thread>
#include <curl/curl.h>
#include <string>
#include <mutex>


class HTTPSyncNode {


	public:

		HTTPSyncNode(DBHandler *db,int delay, bool removeLogs);
        
        bool init();
        bool start();
        void ProcessMessage(const Message* message);


	private:

        bool m_initialised;

		CURL *curl;
		CURLcode m_res;
		bool m_reportedConnectError;
		std::string m_shipID;
		std::string m_shipPWD;
		std::string m_serverURL;
		bool m_removeLogs;
		int m_delay;
		int m_pushOnlyLatestLogs;
		std::mutex m_mutex;

		DBHandler *m_dbHandler;

		std::string getData(std::string call);
        void setShipID(std::string shipID);
		void setShipPWD(std::string shipPWD);
		void setServerURL(std::string URL);

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

#endif
