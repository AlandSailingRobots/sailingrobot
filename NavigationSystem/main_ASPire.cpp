#include <string>
#include "DataBase/DBHandler.h"
#include "DataBase/DBLoggerNode.h"
#include "HTTPSync/HTTPSyncNode.h"
#include "MessageBus/MessageBus.h"
#include "Messages/DataRequestMsg.h"
#include "SystemServices/Logger.h"

#include "Navigation/WaypointMgrNode.h"
#include "WorldState/WindStateNode.h"
#include "WorldState/CollidableMgr/CollidableMgr.h"

#include "LowLevelControllers/LowLevelController.h" // NOTE - Maël: It will change
#include "LowLevelControllers/WingsailControlNode.h"
#include "LowLevelControllers/CourseRegulatorNode.h"

#if LOCAL_NAVIGATION_MODULE == 1
  #include "WorldState/VesselStateNode.h" // NOTE - Maël: It will change
  #include "Navigation/LocalNavigationModule/LocalNavigationModule.h"
  #include "Navigation/LocalNavigationModule/Voters/WaypointVoter.h"
  #include "Navigation/LocalNavigationModule/Voters/WindVoter.h"
  #include "Navigation/LocalNavigationModule/Voters/ChannelVoter.h"
  #include "Navigation/LocalNavigationModule/Voters/ProximityVoter.h"
  #include "Navigation/LocalNavigationModule/Voters/MidRangeVoter.h"
#else
  #include "WorldState/StateEstimationNode.h" // NOTE - Maël: It will change
  #include "Navigation/LineFollowNode.h"
#endif

#if SIMULATION == 1
  #include "Simulation/SimulationNode.h"
#else
  #include "Hardwares/HMC6343Node.h"
  #include "Hardwares/GPSDNode.h"
  #include "Hardwares/CAN_Services/CANService.h"
  #include "Hardwares/CANWindsensorNode.h" 
  #include "Hardwares/ActuatorNodeASPire.h"
#endif


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
    	Logger::info("ASPire");
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

	int dbLoggerWaitTime = 100; 		// wait time (in milliseconds) between messages from the messageBus
	int dbLoggerUpdateFrequency = 1000; // updating frequency to the database (in milliseconds)
	int dbLoggerQueueSize = 5; 			// how many messages to log to the databse at a time
	DBLoggerNode dbLoggerNode(messageBus, dbHandler, dbLoggerWaitTime, dbLoggerUpdateFrequency, dbLoggerQueueSize);

	int dbHandler_delay = dbHandler.retrieveCellAsInt("httpsync_config", "1","delay");
	bool removeLogs = dbHandler.retrieveCellAsInt("httpsync_config","1","remove_logs");
	HTTPSyncNode httpsync(messageBus, &dbHandler, dbHandler_delay, removeLogs);

	WindStateNode windStateNode(messageBus);

	WaypointMgrNode waypoint(messageBus, dbHandler);

	double PGAIN = 0.20;
	double IGAIN = 0.30;
	LowLevelController llc(messageBus, dbHandler, PGAIN, IGAIN); // NOTE - Maël: It will change
	WingsailControlNode wingSailControlNode(messageBus, dbHandler);
	CourseRegulatorNode courseRegulatorNode(messageBus, dbHandler);

  	#if LOCAL_NAVIGATION_MODULE == 1
		VesselStateNode vesselState	( messageBus, 0.2 ); // NOTE - Maël: It will change
		LocalNavigationModule lnm	( messageBus );
		CollidableMgr collidableMgr;

		const int16_t MAX_VOTES = 25;
		WaypointVoter waypointVoter( MAX_VOTES, 1 );
		WindVoter windVoter( MAX_VOTES, 1 );
		ChannelVoter channelVoter( MAX_VOTES, 1 );
		MidRangeVoter midRangeVoter( MAX_VOTES, 1, collidableMgr );
		ProximityVoter proximityVoter( MAX_VOTES, 2, collidableMgr);

		lnm.registerVoter( &waypointVoter );
		lnm.registerVoter( &windVoter );
		lnm.registerVoter( &channelVoter );
		lnm.registerVoter( &proximityVoter );
		lnm.registerVoter( &midRangeVoter );
  	#else
		double vesselStateLoopTime = dbHandler.retrieveCellAsDouble("vesselState_config","1", "loop_time");
	  	StateEstimationNode stateEstimationNode(messageBus, dbHandler,vesselStateLoopTime); // NOTE - Maël: It will change

		LineFollowNode sailingLogic(messageBus, dbHandler);
  	#endif


	#if SIMULATION == 1
	  	#if LOCAL_NAVIGATION_MODULE == 1
	  		SimulationNode simulation(messageBus, &collidableMgr);
	  	#else
			SimulationNode simulation(messageBus);
	  	#endif
  	#else
		CANService canService;

		const int headingBufferSize = dbHandler.retrieveCellAsInt("buffer_config", "1", "compass");
		double compassLoopTime = 0.1;
		HMC6343Node compass(messageBus, dbHandler, headingBufferSize, compassLoopTime);

		double gpsdLoopTime = dbHandler.retrieveCellAsDouble("GPSD_config", "1", "loop_time");
	  	GPSDNode gpsd(messageBus, dbHandler, gpsdLoopTime);

		int time_filter_ms = dbHandler.retrieveCellAsInt("windState_config", "1", "time_filter_ms");
	  	CANWindsensorNode windSensor(messageBus, dbHandler, canService, time_filter_ms);

	  	ActuatorNodeASPire actuators(messageBus, canService);
	#endif


	// Initialise nodes
	//-------------------------------------------------------------------------------

	bool requireNetwork = (bool) (dbHandler.retrieveCellAsInt("sailing_robot_config", "1", "require_network"));
	if (requireNetwork)
	{
		initialiseNode(httpsync, "Httpsync", NodeImportance::CRITICAL);
	}
	else 
	{
		initialiseNode(httpsync, "Httpsync", NodeImportance::NOT_CRITICAL);
	}

	initialiseNode(dbLoggerNode, "DBLogger", NodeImportance::CRITICAL);
	initialiseNode(windStateNode,"WindState",NodeImportance::CRITICAL);
	initialiseNode(waypoint, "Waypoint", NodeImportance::CRITICAL);

	initialiseNode(llc, "Low Level Controller", NodeImportance::CRITICAL); // NOTE - Maël: It will change
 	initialiseNode(wingSailControlNode, "Wing Sail Controller", NodeImportance::CRITICAL);
 	initialiseNode(courseRegulatorNode, "course regulator", NodeImportance::CRITICAL);

	#if LOCAL_NAVIGATION_MODULE == 1
		initialiseNode( vesselState, "Vessel State", NodeImportance::CRITICAL ); // NOTE - Maël: It will change
		initialiseNode( lnm, "Local Navigation Module",	NodeImportance::CRITICAL );
	#else
		initialiseNode(stateEstimationNode,"StateEstimation",NodeImportance::CRITICAL); // NOTE - Maël: It will change
		initialiseNode(sailingLogic, "LineFollow", NodeImportance::CRITICAL);
	#endif

	#if SIMULATION == 1
		initialiseNode(simulation,"Simulation",NodeImportance::CRITICAL);
	#else
		initialiseNode(compass, "Compass", NodeImportance::CRITICAL);
		initialiseNode(gpsd, "GPSD", NodeImportance::CRITICAL);
		initialiseNode(windSensor, "Wind Sensor", NodeImportance::CRITICAL);
		initialiseNode(actuators, "Actuators", NodeImportance::CRITICAL);
	#endif

	// Start active nodes
	//-------------------------------------------------------------------------------

	httpsync.start();
	dbLoggerNode.start();

	#if SIMULATION == 1
		simulation.start();
	#else
		compass.start();
		gpsd.start();
		windSensor.start();
	#endif

	#if LOCAL_NAVIGATION_MODULE == 1
		vesselState.start(); // NOTE - Maël: It will change
		lnm.start();
		collidableMgr.startGC();
	#else
		stateEstimationNode.start(); // NOTE - Maël: It will change
		sailingLogic.start();
	#endif

	wingSailControlNode.start();
	courseRegulatorNode.start();
	//-------------------------------------------------------------------------------

	// Begins running the message bus
	Logger::info("Message bus started!");
	messageBus.run();

	Logger::shutdown();
	exit(0);
}
