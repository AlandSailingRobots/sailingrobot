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


xBeeSync* xbee_handle;


static void threadXBeeSyncRun() {
	xbee_handle->run();

	Logger::warning("Xbee Sync thread has exited");
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
		Logger::warning("Xbee Sync thread has exited");
	}
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
		path = "./";
		db_name = "asr.db";
		errorLog = "errors.log";
	} else {
		path = std::string(argv[1]);
		db_name = "asr.db";
		errorLog = "errors.log";
	}

	printf("================================================================================\n");
	printf("\t\t\t\tSailing Robot\n");
	printf("\n");
	printf("================================================================================\n");

	if (Logger::init()) {
		Logger::info("Built on %s at %s", __DATE__, __TIME__);
		Logger::info("Logger init 		[OK]");
	}
	else {
		Logger::info("Logger init 		[FAILED]");
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

	DBHandler db(path+db_name);

	if(db.initialise())
	{
		Logger::info("Successful database connection established");
	}

	bool mockGPS = db.retrieveCellAsInt("mock","1","gps");
    bool mockWindsensor = db.retrieveCellAsInt("mock","1","windsensor");

	// Create main sailing robot controller
	int http_delay =  db.retrieveCellAsInt("httpsync_config", "1", "delay");
	bool removeLogs = db.retrieveCellAsInt("httpsync_config", "1", "remove_logs");
	//bool removeLogs = true;

	httpsync_handle = new HTTPSync( &db, http_delay, removeLogs );
	
    SailingRobot sr_handle(&externalCommand, &systemstate, &db, httpsync_handle);

	GPSupdater gps_updater(&systemstate,mockGPS);
	gps_handle = &gps_updater;

	try {
		if( not sr_handle.init(path, errorLog) )
		{
			Logger::error("Failed to initialise SailingRobot, exiting...");
			return 1;
		}

		windsensor_handle.reset(
			new WindsensorController(
				&systemstate,
				mockWindsensor,
				db.retrieveCell("windsensor_config", "1", "port"),
				db.retrieveCellAsInt("windsensor_config", "1", "baud_rate"),
				db.retrieveCellAsInt("buffer_config", "1", "windsensor")
			)
		);

		bool xBee_sending = db.retrieveCellAsInt("xbee_config", "1", "send");
		bool xBee_receiving = db.retrieveCellAsInt("xbee_config", "1", "recieve");
		bool xBee_sendLogs = db.retrieveCellAsInt("xbee_config", "1", "send_logs");
		double xBee_loopTime = stod(db.retrieveCell("xbee_config", "1", "loop_time"));

		xbee_handle = new xBeeSync(&externalCommand, &systemstate, &db, xBee_sendLogs, xBee_sending, xBee_receiving,xBee_loopTime);
		
		if(xbee_handle->init())
		{
			// Start xBeeSync thread
			std::unique_ptr<ThreadRAII> xbee_sync_thread;

			if (xBee_sending || xBee_receiving) 
			{
				xbee_sync_thread = std::unique_ptr<ThreadRAII>(new ThreadRAII(std::thread(threadXBeeSyncRun), ThreadRAII::DtorAction::detach));
			}
		}

		// I2CController thread
		bool mockArduino = db.retrieveCellAsInt("mock","1","analog_arduino");
    	bool mockCompass = db.retrieveCellAsInt("mock","1","compass");
		int  headningBufferSize = db.retrieveCellAsInt("buffer_config", "1", "compass");
		double i2cLoopTime = stod(db.retrieveCell("i2c_config", "1", "loop_time"));

		if(mockArduino) { Logger::warning("Using mock Arduino"); }
		if(mockArduino) { Logger::warning("Using mock compass"); }

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

		sr_handle.run();
	} 
	catch (const char * e) {
		printf("ERROR[%s]\n\n",e);
		return 1;
	}


	delete xbee_handle;

	printf("-Finished.\n");
	return 0;
}
