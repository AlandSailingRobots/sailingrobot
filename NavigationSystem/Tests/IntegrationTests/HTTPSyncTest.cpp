#include <string>
#include "Database/DBHandler.h"
#include "Database/DBLoggerNode.h"
#include "HTTPSync/HTTPSyncNode.h"
#include "MessageBus/MessageBus.h"
#include "Messages/DataRequestMsg.h"
#include "SystemServices/Logger.h"


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
	printf("\t\t\t\tSailing Robot\n");
	printf("\n");
	printf("================================================================================\n");


	// This is for eclipse development so the output is constantly pumped out.
	setbuf(stdout, NULL);

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
    	Logger::info("HTTPSyncTest");
	  	#if LOCAL_NAVIGATION_MODULE == 1
			Logger::info( "Using Local Navigation Module" );
	  	#else
			Logger::info( "Using Line-follow" );
	  	#endif
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


	HTTPSyncNode httpsync(messageBus, &dbHandler);

	DBLoggerNode dblogger(messageBus, dbHandler, 5);


	// Initialise nodes
	//-------------------------------------------------------------------------------

	initialiseNode(httpsync, "Httpsync", NodeImportance::CRITICAL);



	// Start active nodes
	//-------------------------------------------------------------------------------

	httpsync.start();
	dblogger.start();

	//-------------------------------------------------------------------------------

	// Begins running the message bus
	Logger::info("Message bus started!");
	messageBus.run();

	Logger::shutdown();
	exit(0);
}
