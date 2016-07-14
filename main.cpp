
#include <string>
#include "logger/Logger.h"
#include "MessageBus.h"
#include "Nodes/MessageLoggerNode.h"
#include "Nodes/CV7Node.h"
#include "Messages/DataRequestMsg.h"
#include "dbhandler/DBHandler.h"

enum class NodeImportance {
	CRITICAL,
	NOT_CRITICAL
};

///----------------------------------------------------------------------------------
/// Initialises a node and shutsdown the program if a critical node fails.
///
/// @param node 			A pointer to the node to initialise
/// @param nodeName 		A string name of the node, for logging purposes.
/// @param importance 		Whether the node is a critcal node or not critical. If a
///							critical node fails to initialise the program will
///							shutdown.
///
///----------------------------------------------------------------------------------
void initialiseNode(Node& node, const char* nodeName, NodeImportance importance)
{
	if(node.init())
	{
		Logger::info("Node: %s - init\t[OK]", nodeName);
	}
	else
	{
		Logger::error("Node: %s - init\t[FAILED]", nodeName);

		if(importance == NodeImportance::CRITICAL)
		{
			Logger::error("Critical node failed to initialise, shutting down");
			Logger::shutdown();
			exit(1);
		}
	}
}

///----------------------------------------------------------------------------------
/// Entry point, can accept one argument containing a relative path to the database.
///
///----------------------------------------------------------------------------------
int main(int argc, char *argv[]) 
{
	// This is for eclipse development so the output is constantly pumped out.
	setbuf(stdout, NULL); 

	// Database Path
	std::string db_path;
	if (argc < 2) {
		db_path = "./asr.db";
	} else {
		db_path = std::string(argv[1]);
	}

	printf("================================================================================\n");
	printf("\t\t\t\tSailing Robot\n");
	printf("\n");
	printf("================================================================================\n");

	if (Logger::init()) {
		Logger::info("Built on %s at %s", __DATE__, __TIME__);
		Logger::info("Logger init\t\t[OK]");
	}
	else {
		Logger::error("Logger init\t\t[FAILED]");
	}

	MessageBus messageBus;
	DBHandler dbHandler(db_path);

	if(dbHandler.initialise())
	{
		Logger::info("Database init\t\t[OK]");
	}
	else
	{
		Logger::error("Database init\t\t[FAILED]");
		Logger::shutdown();
		exit(1);
	}

	// Create nodes
	MessageLoggerNode msgLogger(messageBus);
	CV7Node windSensor(messageBus, dbHandler.retrieveCell("windsensor_config", "1", "port"), dbHandler.retrieveCellAsInt("windsensor_config", "1", "baud_rate"));

	// Initialise nodes
	initialiseNode(msgLogger, "Message Logger", NodeImportance::NOT_CRITICAL);
	initialiseNode(windSensor, "Wind Sensor", NodeImportance::CRITICAL);

	// Start active nodes
	windSensor.start();

	// NOTE - Jordan: Just to ensure messages are following through the system
	DataRequestMsg* dataRequest = new DataRequestMsg(NodeID::MessageLogger);
	messageBus.sendMessage(dataRequest);

	Logger::info("Message bus started!");
	messageBus.run();
	exit(0);
}


// Purely for reference, remove once complete

/*xBeeSync* xbee_handle;


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
	// This is for eclipse development so the output is constantly pumped out.
	setbuf(stdout, NULL); 

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

	// Default time
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
}*/
