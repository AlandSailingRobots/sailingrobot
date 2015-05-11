#include "SailingRobot.h"
#include "thread/SystemState.h"
#include "xBeeSync.h"
#include <thread>

static void threadXBeeSyncRun(xBeeSync *xbee_sync) {
	xbee_sync->run();
}

int main(int argc, char *argv[]) {

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
	SailingRobot sr;

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
		xBeeSync xbee_sync(&systemstate);
		printf("-OK\n");

		printf("-Executing...\n");
		// start xBeeSync thread
		std::thread xbee_sync_thread (threadXBeeSyncRun, &xbee_sync);
		sr.run();
		printf("-DONE\n");

		xbee_sync.close();
		xbee_sync_thread.join();

	} catch (const char * e) {
		printf("ERROR[%s]\n\n",e);
		sr.shutdown();
		return 1;
	}

	sr.shutdown();

	printf("-Finished.\n");
	return 0;
}
