#include "main.h"

#include "SailingRobot.h"
#include "thread/ExternalCommand.h"
#include "thread/SystemState.h"
#include "thread/ThreadRAII.h"
#include "GPSupdater.h"
#include <thread>
#include <unistd.h>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <ctime>

static void threadXBeeSyncRun() {
	try {
		xbee_handle->run();
	} catch (const char * e) {
		std::cout << "ERROR while running static void threadXBeeSyncRun()" << e << std::endl;
	}
}

static void threadGPSupdate() {
	try {
		gps_handle->run();
	}
	catch (const char * e) {
		std::cout << "ERROR while running static void threadGPSupdate()" << e << std::endl;
	}
}

static void threadHTTPSyncRun() {
	try {
	// httpsync_handle->run();
	}
	catch (const char * error) {
		std::cout << "ERROR while running static void threadHTTPSyncRun() : " << error << std::endl;
	}
	std::cout << " httpsync thread exited." << std::endl;
}

static void threadWindsensor() {
	windsensor_handle->run();
	std::cout << " * Windsensor thread exited." << std::endl;
}


int main(int argc, char *argv[]) {

	std::string path, db_name, errorLog;
	if (argc < 2) {
		path = "/root/sailingrobot";
		db_name = "/asr.db";
		errorLog = "/errors.log";
	} else {
		path = std::string(argv[1]);
		db_name = "/asr.db";
		errorLog = "/errors.log";
	}

	printf("\n");
	printf("  Sailing Robot\n");
	printf("=================\n");

	try {
		if (m_logger.init("sailingrobot")) {
			std::cout<< "successfull logger init"<<std::endl;
		}
		else {
			std::cout<< "error in logger init"<<std::endl;
		}
	}
	catch (const char* e) {
		std::cout<< "logger exeption thrown: "<< e <<std::endl;
	}

	/* Default time */
	ExternalCommand externalCommand("1970-04-10T10:53:15.1234Z",true,0,0);
	SystemState systemstate(
		SystemStateModel(
			GPSModel("",PositionModel(0,0),0,0,0,0),
			WindsensorModel(0,0,0),
			CompassModel(0,0,0,AccelerationModel(0,0,0) ),
			0,
			0
		)
	);

	printf("-Creating database connection...\n");
	DBHandler db;
	try {
		db.openDatabase(path+db_name);
	} catch (const char * error) {
		printf("!DB ERROR:%s\n", error);
		std::cout << error << std::endl;
		throw;
	}
	printf("-DONE\n");

	bool mockGPS = db.retriveCellAsInt("mock","1","GPS");
        bool mockWindsensor = db.retriveCellAsInt("mock","1","Windsensor");
        bool mockCompass = db.retriveCellAsInt("mock","1","Compass");
        int  headningBufferSize = db.retriveCellAsInt("buffer_configs", "1", "compass");

	// Create main sailing robot controller
	try {
		sr_handle = std::make_unique<SailingRobot>(&externalCommand, &systemstate, &db);
	} catch (const char * error) {
		printf("!SR INIT ERROR: %s\n", error);
		return 1;
	}

	GPSupdater gps_updater(&systemstate,mockGPS);
	gps_handle = &gps_updater;


	try {
		printf("-Initializing...\n");

		sr_handle->init(path, errorLog);

		printf(" Starting Windsensor\t\t");
		windsensor_handle.reset(
			new WindsensorController(
				&systemstate,
				mockWindsensor,
                                mockCompass,
				db.retriveCell("windsensor_config", "1", "port"),
				db.retriveCellAsInt("windsensor_config", "1", "baud_rate"),
				db.retriveCellAsInt("buffer_configs", "1", "windsensor"),
                                headningBufferSize
			)
		);
		printf("OK\n");

		printf("-DONE\n");

		printf("-Starting threads...\n");

		httpsync_handle.reset(new HTTPSync(&db));

		// Start xBeeSync thread

		//	Hämtar ett heltal (1 eller 0) som visar om xbeen skall skicka och ta emot data.
		bool xBee_sending = db.retriveCellAsInt("xbee_config", "1", "send");
		bool xBee_receiving = db.retriveCellAsInt("xbee_config", "1", "recieve");
		bool xBee_sendLogs = db.retriveCellAsInt("xbee_config", "1", "send_logs");

		std::unique_ptr<ThreadRAII> xbee_sync_thread;
		if (xBee_sending || xBee_receiving) {
			xbee_handle.reset(new xBeeSync(&externalCommand, &systemstate, &db, xBee_sendLogs, xBee_sending, xBee_receiving));
			xbee_sync_thread = std::unique_ptr<ThreadRAII>(
				new ThreadRAII(std::thread(threadXBeeSyncRun), ThreadRAII::DtorAction::detach));
		}

		// Start GPSupdater thread
		ThreadRAII gps_reader_thread(
			std::thread(threadGPSupdate),
			ThreadRAII::DtorAction::detach
		);

		// Start httpsync thread
		httpsync_thread = std::unique_ptr<ThreadRAII>(new ThreadRAII(
			std::thread(threadHTTPSyncRun),
			ThreadRAII::DtorAction::detach
		) );

		// Start windsensor thread
		windsensor_thread = std::unique_ptr<ThreadRAII>(new ThreadRAII(
			std::thread(threadWindsensor),
			ThreadRAII::DtorAction::detach
		) );

		printf("-Starting main loop...\n");
		sr_handle->run();
		printf("-DONE\n");

	} catch (const char * e) {
		printf("ERROR[%s]\n\n",e);
		return 1;
	}

	printf("-Finished.\n");
	return 0;
}
