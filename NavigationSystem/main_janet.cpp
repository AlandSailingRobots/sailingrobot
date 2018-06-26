#include <string>
#include "../Database/DBHandler.h"
#include "../Database/DBLoggerNode.h"
#include "../HTTPSync/HTTPSyncNode.h"
#include "../MessageBus/MessageBus.h"
#include "../Messages/DataRequestMsg.h"
#include "../SystemServices/Logger.h"

#include "../Navigation/WaypointMgrNode.h"
#include "../WorldState/StateEstimationNode.h"
#include "../WorldState/WindStateNode.h"
#include "../WorldState/CollidableMgr/CollidableMgr.h"

#include "../LowLevelControllers/SailControlNode.h"
#include "../LowLevelControllers/CourseRegulatorNode.h"

#if LOCAL_NAVIGATION_MODULE == 1
  #include "../Navigation/LocalNavigationModule/LocalNavigationModule.h"
  #include "../Navigation/LocalNavigationModule/Voters/WaypointVoter.h"
  #include "../Navigation/LocalNavigationModule/Voters/WindVoter.h"
  #include "../Navigation/LocalNavigationModule/Voters/ChannelVoter.h"
  #include "../Navigation/LocalNavigationModule/Voters/ProximityVoter.h"
  #include "../Navigation/LocalNavigationModule/Voters/MidRangeVoter.h"
#else
  #include "../Navigation/LineFollowNode.h"
#endif

#if SIMULATION == 1
  #include "../Simulation/SimulationNode.h"
#else
  #include "../Hardwares/ArduinoNode.h"
  #include "../Hardwares/CV7Node.h"
  #include "../Hardwares/HMC6343Node.h"
  #include "../Hardwares/GPSDNode.h"
  #include "ActuatorNode-notused.h" // NOTE - Maël: It will change (to ActuatorNodeJanet.h)
  #include "../Hardwares/MaestroController/MaestroController.h"
  #include "../Xbee/Xbee.h"
  #include "../Xbee/XbeeSyncNode.h"
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
    	Logger::info("Janet");
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

	int dbLoggerQueueSize = 5; 			// how many messages to log to the databse at a time
	DBLoggerNode dbLoggerNode(messageBus, dbHandler, dbLoggerQueueSize);

	//HTTPSyncNode httpsync(messageBus, &dbHandler);

	StateEstimationNode stateEstimationNode(messageBus, dbHandler);
	WindStateNode windStateNode(messageBus);
	WaypointMgrNode waypoint(messageBus, dbHandler);
	CollidableMgr collidableMgr;

	SailControlNode sailControlNode(messageBus, dbHandler);
	CourseRegulatorNode courseRegulatorNode(messageBus, dbHandler);

  	#if LOCAL_NAVIGATION_MODULE == 1
        LocalNavigationModule lnm	( messageBus, dbHandler );

        const int16_t MAX_VOTES = dbHandler.retrieveCellAsInt("config_voter_system","1","max_vote");
		WaypointVoter waypointVoter( MAX_VOTES, dbHandler.retrieveCellAsDouble("config_voter_system","1","waypoint_voter_weight")); // weight = 1
		WindVoter windVoter( MAX_VOTES, dbHandler.retrieveCellAsDouble("config_voter_system","1","wind_voter_weight")); // weight = 1
		ChannelVoter channelVoter( MAX_VOTES, dbHandler.retrieveCellAsDouble("config_voter_system","1","channel_voter_weight")); // weight = 1
		MidRangeVoter midRangeVoter( MAX_VOTES, dbHandler.retrieveCellAsDouble("config_voter_system","1","midrange_voter_weight"), collidableMgr );
		ProximityVoter proximityVoter( MAX_VOTES, dbHandler.retrieveCellAsDouble("config_voter_system","1","proximity_voter_weight"), collidableMgr);

		lnm.registerVoter( &waypointVoter );
		lnm.registerVoter( &windVoter );
		lnm.registerVoter( &channelVoter );
		lnm.registerVoter( &proximityVoter );
		lnm.registerVoter( &midRangeVoter );
  	#else
		LineFollowNode sailingLogic(messageBus, dbHandler);
  	#endif


	#if SIMULATION == 1
	  	SimulationNode simulation(messageBus, 0, &collidableMgr);
  	#else
		CV7Node windSensor(messageBus, dbHandler);
		HMC6343Node compass(messageBus, dbHandler);
	  	GPSDNode gpsd(messageBus, dbHandler);
        ArduinoNode arduino(messageBus, dbHandler);

		int channel = 3;
		int speed = 0;
		int acceleration = 0;
		ActuatorNode sail(messageBus, NodeID::SailActuator, channel, speed, acceleration);

		channel = 4;
		speed = 0;
		acceleration = 0;
		ActuatorNode rudder(messageBus, NodeID::RudderActuator, channel, speed, acceleration);

		MaestroController::init(dbHandler.retrieveCell("config_maestro_controller", "1", "port"));

		XbeeSyncNode xbee(messageBus, dbHandler);
	#endif


	// Initialise nodes
	//-------------------------------------------------------------------------------

	//initialiseNode(httpsync, "Httpsync", NodeImportance::NOT_CRITICAL); // This node is not critical during the developement phase.
	initialiseNode(dbLoggerNode, "DBLogger", NodeImportance::CRITICAL);

	initialiseNode(stateEstimationNode,"StateEstimation",NodeImportance::CRITICAL);
	initialiseNode(windStateNode,"WindState",NodeImportance::CRITICAL);
	initialiseNode(waypoint, "Waypoint", NodeImportance::CRITICAL);

 	initialiseNode(sailControlNode, "Sail Controller", NodeImportance::CRITICAL);
 	initialiseNode(courseRegulatorNode, "Course Regulator", NodeImportance::CRITICAL);

	#if LOCAL_NAVIGATION_MODULE == 1
		initialiseNode( lnm, "Local Navigation Module",	NodeImportance::CRITICAL );
	#else
		initialiseNode(sailingLogic, "LineFollow", NodeImportance::CRITICAL);
	#endif

	#if SIMULATION == 1
		initialiseNode(simulation,"Simulation",NodeImportance::CRITICAL);
	#else
		initialiseNode(windSensor, "Wind Sensor", NodeImportance::CRITICAL);
		initialiseNode(compass, "Compass", NodeImportance::CRITICAL);
		initialiseNode(gpsd, "GPSD", NodeImportance::CRITICAL);
		initialiseNode(arduino, "Arduino", NodeImportance::NOT_CRITICAL);
		initialiseNode(sail, "Sail Actuator", NodeImportance::CRITICAL);
		initialiseNode(rudder, "Rudder Actuator", NodeImportance::CRITICAL);
		initialiseNode(xbee, "Xbee Sync", NodeImportance::NOT_CRITICAL);
	#endif

	// Start active nodes
	//-------------------------------------------------------------------------------

	//httpsync.start();
	dbLoggerNode.start();

	stateEstimationNode.start();
	collidableMgr.startGC();

	sailControlNode.start();
	courseRegulatorNode.start();

	#if SIMULATION == 1
		simulation.start();
	#else
		windSensor.start();
		compass.start();
		gpsd.start();
		arduino.start();
	#endif

	#if LOCAL_NAVIGATION_MODULE == 1
		lnm.start();
	#else
		sailingLogic.start();
	#endif

	// Begins running the message bus
	//-------------------------------------------------------------------------------
	Logger::info("Message bus started!");
	messageBus.run();


	Logger::shutdown();
	exit(0);
}
