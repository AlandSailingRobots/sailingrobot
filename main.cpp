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
		httpsync_handle->run();
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

static void threadI2CController() {
	try {
		i2cController_handle->run();
	}
	catch (const char * error) {
		std::cout << "ERROR while running static void threadI2CController() : " << error << std::endl;
	}
	std::cout << " I2Ccontroller thread exited." << std::endl;
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
			AnalogArduinoModel(0, 0, 0, 0),
			0,
			0
		)
	);

	printf("-Creating database...\n");
	DBHandler db(path+db_name);
	printf("-DONE\n");

	bool mockGPS = db.retrieveCellAsInt("mock","1","gps");
    bool mockWindsensor = db.retrieveCellAsInt("mock","1","windsensor");

	// Create main sailing robot controller
    SailingRobot sr_handle(&externalCommand, &systemstate, &db);

	GPSupdater gps_updater(&systemstate,mockGPS);
	gps_handle = &gps_updater;

	try {
		printf("-Initializing...\n");

		sr_handle.init(path, errorLog);

		printf(" Starting Windsensor\t\t");
		windsensor_handle.reset(
			new WindsensorController(
				&systemstate,
				mockWindsensor,
				db.retrieveCell("windsensor_config", "1", "port"),
				db.retrieveCellAsInt("windsensor_config", "1", "baud_rate"),
				db.retrieveCellAsInt("buffer_config", "1", "windsensor")
			)
		);
		printf("OK\n");

		printf("-DONE\n");

		printf("-Starting threads...\n");

		int http_delay =  db.retrieveCellAsInt("httpsync_config", "1", "delay");
		bool removeLogs = db.retrieveCellAsInt("httpsync_config", "1", "remove_logs");
		//bool removeLogs = true;

		httpsync_handle.reset(new HTTPSync( &db, http_delay, removeLogs ));

		bool xBee_sending = db.retrieveCellAsInt("xbee_config", "1", "send");
		bool xBee_receiving = db.retrieveCellAsInt("xbee_config", "1", "recieve");
		bool xBee_sendLogs = db.retrieveCellAsInt("xbee_config", "1", "send_logs");
		double xBee_loopTime = stod(db.retrieveCell("xbee_config", "1", "loop_time"));
		
		// Start xBeeSync thread
		std::unique_ptr<ThreadRAII> xbee_sync_thread;
		if (xBee_sending || xBee_receiving) {
			xbee_handle.reset(new xBeeSync(&externalCommand, &systemstate, &db, xBee_sendLogs, xBee_sending, xBee_receiving,xBee_loopTime));
			xbee_sync_thread = std::unique_ptr<ThreadRAII>(
				new ThreadRAII(std::thread(threadXBeeSyncRun), ThreadRAII::DtorAction::detach));
		}

		// I2CController thread
		bool mockArduino = db.retrieveCellAsInt("mock","1","analog_arduino");
    	bool mockCompass = db.retrieveCellAsInt("mock","1","compass");
		int  headningBufferSize = db.retrieveCellAsInt("buffer_config", "1", "compass");
		double i2cLoopTime = stod(db.retrieveCell("i2c_config", "1", "loop_time"));

		i2cController_handle.reset(new I2CController(&systemstate, mockArduino, mockCompass, headningBufferSize, i2cLoopTime));
		i2cController_handle->init();

		// Start i2cController thread
		i2cController_thread = std::unique_ptr<ThreadRAII>(new ThreadRAII(
			std::thread(threadI2CController),
			ThreadRAII::DtorAction::detach
		) );

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
		sr_handle.run();
		printf("-DONE\n");

	} catch (const char * e) {
		printf("ERROR[%s]\n\n",e);
		return 1;
	}

	printf("-Finished.\n");
	return 0;
}
