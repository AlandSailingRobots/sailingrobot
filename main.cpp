#include "SailingRobot.h"
#include "thread/SystemState.h"
#include "xBeeSync.h"
#include "GPSupdater.h"
#include "global.h"
#include <thread>
#include <unistd.h>
#include <signal.h>

static void threadXBeeSyncRun() {
	xbee_handle->run();
}

static void threadGPSupdate() {
	gps_handle->run();
}

void term(int signum)
{
	printf("\n-SIGINT detected, trying to exit cleanly...\n");
	printf(" stopping main loop\n");
	sr_handle->shutdown();
	printf(" stopping xBee thread\n");
	xbee_handle->close();
	printf(" stopping GPS thread\n");
	gps_handle->close();
	printf("-DONE\n");
}

int main(int argc, char *argv[]) {

	std::string path, db_name, errorLog;
	if (argc < 2) {
		path = "/root/sailingrobot/";
		db_name = "asr.db";
		errorLog = "errors.log";
	} else {
		path = std::string(argv[1]);
		db_name = "/asr.db";
		errorLog = "/errors.log";
	}

    struct sigaction action;
    memset(&action, 0, sizeof(struct sigaction));
    action.sa_handler = term;
    sigaction(SIGINT, &action, NULL);

	printf("\n");
	printf("  Sailing Robot\n");
	printf("=================\n");

	SystemState systemstate(
		SystemStateModel(
			GPSModel("",0,0,0,0,0,0),
			WindsensorModel(0,0,0),
			CompassModel(0,0,0),
			0,
			0
		)
	);
	GPSReader gps_reader;

	DBHandler db;
	try {
		db.openDatabase(path+db_name);
	} catch (const char * error) {
		printf("!DB ERROR:%s\n", error);
		throw;
	}
	db_handle = &db;

	// Create main sailing robot controller
	SailingRobot sr(&systemstate,&gps_reader,&db);
	sr_handle = &sr;

	// Create thread controllers
	xBeeSync xbee_sync(&systemstate);
	xbee_handle = &xbee_sync;
	GPSupdater gps_updater(&gps_reader);
	gps_handle = &gps_updater;

	try {
		printf("-Initializing...\n");
		sr.init(path, errorLog);
		printf("-DONE\n");

		printf("-Starting threads...\n");
		//start xBeeSync thread
		std::thread xbee_sync_thread (threadXBeeSyncRun);
		printf(" xBee thread started\n");
		//start GPSupdater thread
		std::thread gps_reader_thread (threadGPSupdate);
		printf("-Starting main loop...\n");
		sr.run();
		printf("-DONE\n");

	} catch (const char * e) {
		printf("ERROR[%s]\n\n",e);
		term(1);
		//sr.shutdown();
		return 1;
	}

	printf("-Finished.\n");
	return 0;
}
