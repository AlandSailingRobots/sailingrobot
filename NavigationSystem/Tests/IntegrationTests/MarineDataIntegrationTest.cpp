#include <string>
#include <CourseRegulatorNode.h>
#include <WingsailControlNode.h>
#include <CollidableMgr/CollidableMgr.h>
#include <WaypointMgrNode.h>
#include <WindStateNode.h>
#include <StateEstimationNode.h>
#include <HTTPSyncNode.h>
#include <DBLoggerNode.h>
#include <Logger.h>
#include <CAN_Services/CANService.h>
#include <GPSDNode.h>
#include <CANMarineSensorReceiver.h>
#include <CANMarineSensorTransmissionNode.h>
#include <DBHandler.h>
#include <MessageBus.h>
#include "DataBase/DBHandler.h"
#include "DataBase/DBLoggerNode.h"
#include "HTTPSync/HTTPSyncNode.h"
#include "MessageBus/MessageBus.h"
#include "Messages/DataRequestMsg.h"
#include "SystemServices/Logger.h"




#include "Hardwares/GPSDNode.h"
#include "Hardwares/CAN_Services/CANService.h"
#include "Hardwares/CANArduinoNode.h"

#include "Hardwares/CANMarineSensorReceiver.h"
#include "Hardwares/CANMarineSensorTransmissionNode.h"




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
	printf("================================================================================\n");
	printf("\t\t\t\tSailing Robot Marine Sensor Integration Test\n");
	printf("\n");
	printf("================================================================================\n");

	// Database Path
	std::string db_path;
	if (argc < 2)
	{
		db_path = "../asr.db";
	}
	else
	{
		db_path = std::string(argv[1]);
	}

	// Declare DBHandler and MessageBus
	DBHandler dbHandler(db_path);
	MessageBus messageBus;

	// Initialise logger
	if (Logger::init())
	{
		Logger::info("Built on %s at %s", __DATE__, __TIME__);
    	Logger::info("Marine Sensor Integration Test");
		Logger::info("Logger init\t\t[OK]");
	}
	else
	{
		Logger::error("Logger init\t\t[FAILED]");
	}

	// Initialise DBHandler
	if(dbHandler.initialise())
	{
		Logger::info("Database Handler init\t\t[OK]");
	}
	else
	{
		Logger::error("Database Handler init\t\t[FAILED]");
		Logger::shutdown();
		exit(1);
	}


	// Declare nodes
	//-------------------------------------------------------------------------------

	int dbLoggerQueueSize = 5; 			// how many messages to log to the databse at a time
	DBLoggerNode dbLoggerNode(messageBus, dbHandler, dbLoggerQueueSize);
	HTTPSyncNode httpsync(messageBus, &dbHandler);





	CANService canService;

	GPSDNode gpsd(messageBus, dbHandler);
	CANMarineSensorReceiver canMarineSensorReciver(messageBus, canService);
	CANMarineSensorTransmissionNode canMarineSensorTransmissionNode(messageBus, canService);


	// Initialise nodes
	//-------------------------------------------------------------------------------

	initialiseNode(httpsync, "Httpsync", NodeImportance::NOT_CRITICAL); // This node is not critical during the developement phase.
	initialiseNode(dbLoggerNode, "DBLogger", NodeImportance::CRITICAL);

    initialiseNode(gpsd, "GPSD", NodeImportance::CRITICAL);
    initialiseNode(canMarineSensorTransmissionNode, "Marine Sensors", NodeImportance::NOT_CRITICAL);


	// Start active nodes
	//-------------------------------------------------------------------------------

	httpsync.start();
	dbLoggerNode.start();


    auto future = canService.start();
    gpsd.start();


	// Begins running the message bus
	//-------------------------------------------------------------------------------
	Logger::info("Message bus started!");
	messageBus.run();


	Logger::shutdown();
	exit(0);
}
