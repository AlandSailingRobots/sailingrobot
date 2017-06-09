
#include <string>
#include "SystemServices/Logger.h"
#include "MessageBus/MessageBus.h"
#include "Nodes/MessageLoggerNode.h"

#if SIMULATION == 1
 #include "Nodes/SimulationNode.h"
#else
 #include "Nodes/CV7Node.h"
 #include "Nodes/HMC6343Node.h"
 #include "Nodes/GPSDNode.h"
 #include "Nodes/ActuatorNode.h"
 #include "Nodes/ArduinoNode.h"
#endif

#include "Nodes/WaypointMgrNode.h"
#include "Nodes/StateEstimationNode.h"
#include "Nodes/WindStateNode.h"
#include "Nodes/HTTPSyncNode.h"
#include "Nodes/XbeeSyncNode.h"
#include "Nodes/LineFollowNode.h"
#include "Nodes/LowLevelControllerNodeJanet.h"


#if USE_OPENCV_COLOR_DETECTION == 1
#include "Nodes/obstacledetection/colorDetectionNode.h"
#endif

#include "Messages/DataRequestMsg.h"
#include "dbhandler/DBHandler.h"
#include "Nodes/DBLoggerNode.h"
#include "HardwareServices/MaestroController/MaestroController.h"
#include "xBee/Xbee.h"


#define DISABLE_LOGGING 0

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
		Logger::error("Node: %s - init\t\t[FAILED]", nodeName);

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

		#if SIMULATION == 1
		printf("using simulation\n");
		SimulationNode simulation(messageBus, 0.5);

		#else

		XbeeSyncNode xbee(messageBus, dbHandler);
		CV7Node windSensor(messageBus, dbHandler.retrieveCell("windsensor_config", "1", "port"), dbHandler.retrieveCellAsInt("windsensor_config", "1", "baud_rate"));
		HMC6343Node compass(messageBus, dbHandler.retrieveCellAsInt("buffer_config", "1", "compass"));
    //NOTE: the second parameter (sleep time in seconds) should probably be read from the database
    GPSDNode gpsd(messageBus, 0.5);
    //NOTE: the second parameter (sleep time in seconds) should probably be read from the database
		ArduinoNode arduino(messageBus, 0.1);
		std::vector<std::string> list;
		list.push_back("red");
		#endif


		HTTPSyncNode httpsync(messageBus, &dbHandler, 0, false);
	    StateEstimationNode stateEstimationNode(messageBus, .5, .5);
    	WindStateNode windStateNode(messageBus, 500);
		WaypointMgrNode waypoint(messageBus, dbHandler);


		ActiveNode* sailingLogic;
    	Node* lowLevelControllerNodeJanet;

		sailingLogic = new LineFollowNode(messageBus, dbHandler);
    	lowLevelControllerNodeJanet = new LowLevelControllerNodeJanet(messageBus, 30, 60, dbHandler);

		#if SIMULATION == 0
		int channel = dbHandler.retrieveCellAsInt("sail_servo_config", "1", "channel");
		int speed = dbHandler.retrieveCellAsInt("sail_servo_config", "1", "speed");
		int acceleration = dbHandler.retrieveCellAsInt("sail_servo_config", "1", "acceleration");

		ActuatorNode sail(messageBus, NodeID::SailActuator, channel, speed, acceleration);

		channel = dbHandler.retrieveCellAsInt("rudder_servo_config", "1", "channel");
		speed = dbHandler.retrieveCellAsInt("rudder_servo_config", "1", "speed");
		acceleration = dbHandler.retrieveCellAsInt("rudder_servo_config", "1", "acceleration");

		ActuatorNode rudder(messageBus, NodeID::RudderActuator, channel, speed, acceleration);
		MaestroController::init(dbHandler.retrieveCell("maestro_controller_config", "1", "port"));
		#endif
		bool requireNetwork = (bool) (dbHandler.retrieveCellAsInt("sailing_robot_config", "1", "require_network"));

		// System services


		// Initialise nodes
		initialiseNode(msgLogger, "Message Logger", NodeImportance::NOT_CRITICAL);

		#if SIMULATION == 1
		initialiseNode(simulation,"Simulation Node",NodeImportance::CRITICAL);
		#else
		initialiseNode(xbee, "Xbee Sync Node", NodeImportance::NOT_CRITICAL);
		initialiseNode(windSensor, "Wind Sensor", NodeImportance::CRITICAL);
		initialiseNode(compass, "Compass", NodeImportance::CRITICAL);
		initialiseNode(gpsd, "GPSD Node", NodeImportance::CRITICAL);
		initialiseNode(sail, "Sail Actuator", NodeImportance::CRITICAL);
		initialiseNode(rudder, "Rudder Actuator", NodeImportance::CRITICAL);
		initialiseNode(arduino, "Arduino Node", NodeImportance::NOT_CRITICAL);
		initialiseNode(colorDetection, "Colour detection node", NodeImportance::NOT_CRITICAL);
		#endif

		if (requireNetwork)
		{
			initialiseNode(httpsync, "Httpsync Node", NodeImportance::CRITICAL);
		}
		else
		{
			initialiseNode(httpsync, "Httpsync Node", NodeImportance::NOT_CRITICAL);
		}

		//initialiseNode(vessel, "Vessel State Node", NodeImportance::CRITICAL);
	    initialiseNode(stateEstimationNode,"StateEstimation Node",NodeImportance::CRITICAL);
    	initialiseNode(windStateNode,"WindState Node",NodeImportance::CRITICAL);
		initialiseNode(waypoint, "Waypoint Node", NodeImportance::CRITICAL);
		initialiseNode(*sailingLogic, "LineFollow Node", NodeImportance::CRITICAL);
    	initialiseNode(*lowLevelControllerNodeJanet, "LowLevelControllerNodeJanet Node", NodeImportance::CRITICAL);

		// Start active nodes
		#if SIMULATION == 1
		simulation.start();
		#else
		xbee.start();
		windSensor.start();
		compass.start();
		gpsd.start();
		arduino.start();
		#endif

		httpsync.start();
		stateEstimationNode.start();
		sailingLogic->start();

		// NOTE - Jordan: Just to ensure messages are following through the system
		MessagePtr dataRequest = std::make_unique<DataRequestMsg>(NodeID::MessageLogger);
		messageBus.sendMessage(std::move(dataRequest));

		int dbLoggerWaitTime = 100; 		// wait time (in milliseconds) between messages from the messageBus
		int dbLoggerUpdateFrequency = 1000; // updating frequency to the database (in milliseconds)
		int dbLoggerQueueSize = 5; 			// how many messages to log to the databse at a time

		DBLoggerNode dbLoggerNode(messageBus, dbHandler, dbLoggerWaitTime, dbLoggerUpdateFrequency, dbLoggerQueueSize);
		initialiseNode(dbLoggerNode, "DBLoggerNode", NodeImportance::CRITICAL);
		dbLoggerNode.start();

		Logger::info("Message bus started!");
		messageBus.run();


		Logger::shutdown();
		delete sailingLogic;
		delete lowLevelControllerNodeJanet;
		exit(0);
}
