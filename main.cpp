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
	std::cout << "SIGINT detected, trying to exit cleanly.." << std::endl;

	xbee_handle->close();
	gps_handle->close();

	sr_handle->shutdown();

}

int main(int argc, char *argv[]) {

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
	GPSReader gps_r;

	// Create main sailing robot controller
	SailingRobot sr(&systemstate,&gps_r);
	sr_handle = &sr;

	// Create thread controller
	xBeeSync xbee_sync(&systemstate);
	xbee_handle = &xbee_sync;
	GPSupdater gps_u(&gps_r);
	gps_handle = &gps_u;

	std::string path, db, errorLog;
	if (argc < 2) {
		path = "/root/sailingrobot/";
		db = "asr.db";
		errorLog = "errors.log";
	} else {
		path = std::string(argv[1]);
		db = "/asr.db";
		errorLog = "/errors.log";
	}

	try {
		printf("-Initializing...\n");
		sr.init(path, db, errorLog);

		printf("-DONE\n");

		printf("-Executing...\n");
		//start xBeeSync thread
		std::thread xbee_sync_thread (threadXBeeSyncRun);
		printf("xBee thread started\n");		
		//start GPSupdater thread
		std::thread gps_reader_thread (threadGPSupdate);

		sr.run();
		printf("-DONE\n");

		//xbee_sync.close();
		//xbee_sync_thread.join();
		//gps_reader_thread.join();

	} catch (const char * e) {
		printf("ERROR[%s]\n\n",e);
		term(1);
		//sr.shutdown();
		return 1;
	}

	printf("-Finished.\n");
	return 0;
}
