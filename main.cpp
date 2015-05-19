#include "SailingRobot.h"
#include "thread/SystemState.h"
#include "xBeeSync.h"
#include "GPSupdater.h"
#include <thread>
#include <unistd.h>

static void threadXBeeSyncRun(xBeeSync *xbee_sync) {
	xbee_sync->run();
}

static void threadGPSupdate(GPSupdater *gps_updater) {
	gps_updater->run();
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
	GPSReader gps_r;
	SailingRobot sr(&systemstate);

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

		//* måste skicka in &gps_r till sr.init så den får tillgång till GPSen.
		//* samt ändra så pekaren används inuti SailingRobot klassen.
		sr.init(path, db, errorLog);

		// Thread objects
		xBeeSync xbee_sync(&systemstate);
		GPSupdater gps_u(&gps_r);

		printf("-OK\n");

		printf("-Executing...\n");
		//start xBeeSync thread
		std::thread xbee_sync_thread (threadXBeeSyncRun, &xbee_sync);
		printf("xBee thread started\n");		
		//start GPSupdater thread
		std::thread gps_reader_thread (threadGPSupdate, &gps_u);
		printf("GPSreader thread started \n");

		sr.run();
		printf("-DONE\n");

		//xbee_sync.close();
		xbee_sync_thread.join();
		gps_reader_thread.join();

	} catch (const char * e) {
		printf("ERROR[%s]\n\n",e);
		sr.shutdown();
		return 1;
	}

	sr.shutdown();

	printf("-Finished.\n");
	return 0;
}
