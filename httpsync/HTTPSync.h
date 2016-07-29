#ifndef __HTTPSYNC_H__
#define __HTTPSYNC_H__

#include "../dbhandler/DBHandler.h"
#include "logger/Logger.h"
#include "models/SystemStateModel.h"

#include <chrono>
#include <thread>
#include <curl/curl.h>
#include <string>
#include <mutex>

typedef void (*waypointUpdateCallback) ();


class HTTPSync {



	public:

		HTTPSync(DBHandler *db,int delay, bool removeLogs);
		~HTTPSync();

		void run();
		void setupHTTPSync();
		void pushDatalogs();
		void updateState();
		void pushWaypoints();
		void pushConfigs();

		void setShipID(std::string shipID);
		void setShipPWD(std::string shipPWD);
		void setServerURL(std::string URL);
		void setWaypointUpdateCallback(waypointUpdateCallback callBack);

	private:

		CURL *curl;
		CURLcode res;
		bool reportedConnectError;
		std::string shipID;
		std::string shipPWD;
		std::string serverURL;
		bool m_running;
		bool m_removeLogs;
		int m_delay;
		int m_pushOnlyLatestLogs;
		std::mutex m_mutex;

		waypointUpdateCallback m_callBack;

		DBHandler *m_dbHandler;


		std::string getData(std::string call);
		void updateConfigs();
		void updateWaypoints();



		bool performCURLCall(std::string data, std::string call, std::string& response);
		bool checkIfNewConfig();
		bool checkIfNewWaypoints();
		bool isRunning();
		void close();
};

#endif
