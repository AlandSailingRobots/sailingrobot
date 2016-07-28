
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

		virtual ~HTTPSyncNode() { }

		///----------------------------------------------------------------------------------
		/// Retrieves server settings from database and initialises curl
		///
		///----------------------------------------------------------------------------------
        bool init();
        void start();
		        
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
        static void HTTPSyncThread(void* nodePtr);


		///----------------------------------------------------------------------------------
		/// Convenience function: creates curl call from argument and returns response (json data)
		///----------------------------------------------------------------------------------
		std::string getData(std::string call);

        bool m_initialised;

		std::string m_shipID;
		std::string m_shipPWD;
		std::string m_serverURL;

		CURL *curl;
		CURLcode m_res;
		bool m_reportedConnectError;

		///----------------------------------------------------------------------------------
		/// Determines wether or not to clear all local logs after a successful push to server
		///----------------------------------------------------------------------------------
		bool m_removeLogs;
		int m_delay;
		int m_pushOnlyLatestLogs;

		DBHandler *m_dbHandler;

};

