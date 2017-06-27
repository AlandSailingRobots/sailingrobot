#include <string>
#include "SystemServices/Logger.h"
#include "MessageBus/MessageBus.h"

#if SIMULATION == 1
 #include "Simulation/SimulationNode.h"
#else
 #include "Hardwares/HMC6343Node.h"
 #include "Hardwares/GPSDNode.h"
 #include "Hardwares/ActuatorNodeASPire.h"
 #include "Hardwares/CAN_Services/CANService.h"
#endif

#include "Hardwares/CANWindsensorNode.h"
#include "Navigation/WaypointMgrNode.h"
#include "WorldState/StateEstimationNode.h"
#include "WorldState/WindStateNode.h"
#include "HTTPSync/HTTPSyncNode.h"
#include "Xbee/XbeeSyncNode.h"
#include "WorldState/VesselStateNode.h"

#if LOCAL_NAVIGATION_MODULE == 1
  #include "Navigation/LocalNavigationModule/LocalNavigationModule.h"
  #include "Navigation/LocalNavigationModule/Voters/WaypointVoter.h"
  #include "Navigation/LocalNavigationModule/Voters/WindVoter.h"
  #include "Navigation/LocalNavigationModule/Voters/ChannelVoter.h"
  #include "Navigation/LocalNavigationModule/Voters/ProximityVoter.h"
  #include "Navigation/LocalNavigationModule/Voters/MidRangeVoter.h"
#else
  #include "Navigation/LineFollowNode.h"
#endif
#include "LowLevelControllers/LowLevelControllerNodeASPire.h"

#include "Messages/DataRequestMsg.h"
#include "DataBase/DBHandler.h"
#include "DataBase/DBLoggerNode.h"
#include "Xbee/Xbee.h"

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

	std::string db_path;
	if (argc < 2) {
		db_path = "../asr.db";
	} else {
		db_path = std::string(argv[1]);
	}

	printf("================================================================================\n");
	printf("\t\t\t\tSailing Robot\n");
	printf("\n");
	printf("================================================================================\n");

	if (Logger::init()) {
		Logger::info("Built on %s at %s", __DATE__, __TIME__);
    Logger::info("ASPire");
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

  #if LOCAL_NAVIGATION_MODULE == 1
  const int16_t MAX_VOTES = 25;
  Logger::info( "Using Local Navigation Module" );
  #else
  Logger::info( "Using Line-follow" );
  #endif

  CANService canService;

  #if LOCAL_NAVIGATION_MODULE == 1
  LocalNavigationModule lnm	( messageBus );
  VesselStateNode vesselState	( messageBus, 0.2 );
  CollidableMgr collidableMgr;
  #else
	ActiveNode* sailingLogic;
  Node* lowLevelControllerNodeASPire;
	sailingLogic = new LineFollowNode(messageBus, dbHandler);
  lowLevelControllerNodeASPire = new LowLevelControllerNodeASPire(messageBus, canService, 45, 30, 50, 10); // NOTE: Values to be read from db
  #endif

	#if SIMULATION == 1
  #if LOCAL_NAVIGATION_MODULE == 1
  SimulationNode 	simulation	( messageBus, &collidableMgr );
  #else
	SimulationNode 	simulation	( messageBus );
  #endif
	#else

	XbeeSyncNode xbee(messageBus, dbHandler);

  CANWindsensorNode windSensor(messageBus, canService,
                                dbHandler.retrieveCellAsInt("windState_config", "1", "time_filter_ms"));
	HMC6343Node compass(messageBus,
                      dbHandler.retrieveCellAsInt("buffer_config", "1", "compass"));
  GPSDNode gpsd(messageBus,
                dbHandler.retrieveCellAsDouble("gpsd_config", "1", "loop_time"));

	#endif

  HTTPSyncNode httpsync(messageBus, &dbHandler,
                        dbHandler.retrieveCellAsInt("httpsync_config", "1", "delay"),
                        dbHandler.retrieveCellAsInt("httpsync_config", "1", "remove_logs"));
  StateEstimationNode stateEstimationNode(messageBus,
                                          dbHandler.retrieveCellAsDouble("vesselState_config", "1", "loop_time"),
                                          dbHandler.retrieveCellAsDouble("vesselState_config", "1", "speedLimit"));
  WindStateNode windStateNode(messageBus,
                              dbHandler.retrieveCellAsDouble("windState_config", "1", "time_filter_ms"));
	WaypointMgrNode waypoint(messageBus, dbHandler);



	#if SIMULATION == 0
	ActuatorNodeASPire sail(messageBus, canService);
	ActuatorNodeASPire rudder(messageBus, canService);
	#endif

	bool requireNetwork = (bool) (dbHandler.retrieveCellAsInt("sailing_robot_config", "1", "require_network"));

	#if SIMULATION == 1
	initialiseNode(simulation,"Simulation Node",NodeImportance::CRITICAL);
	#else
	initialiseNode(xbee, "Xbee Sync Node", NodeImportance::NOT_CRITICAL);
	initialiseNode(windSensor, "Wind Sensor", NodeImportance::CRITICAL);
	initialiseNode(compass, "Compass", NodeImportance::CRITICAL);
	initialiseNode(gpsd, "GPSD Node", NodeImportance::CRITICAL);
	initialiseNode(sail, "Sail Actuator", NodeImportance::CRITICAL);
	initialiseNode(rudder, "Rudder Actuator", NodeImportance::CRITICAL);
	#endif

	if (requireNetwork) {
		initialiseNode(httpsync, "Httpsync Node", NodeImportance::CRITICAL);
	}
	else {
		initialiseNode(httpsync, "Httpsync Node", NodeImportance::NOT_CRITICAL);
	}

	//initialiseNode(vessel, "Vessel State Node", NodeImportance::CRITICAL); // NOTE: Leave for now
  initialiseNode(stateEstimationNode,"StateEstimation Node",NodeImportance::CRITICAL);
	initialiseNode(windStateNode,"WindState Node",NodeImportance::CRITICAL);
	initialiseNode(waypoint, "Waypoint Node", NodeImportance::CRITICAL);
  #if LOCAL_NAVIGATION_MODULE == 1
  initialiseNode( vesselState, 	"Vessel State Node", 		NodeImportance::CRITICAL );
  initialiseNode( lnm,			"Local Navigation Module",	NodeImportance::CRITICAL );

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
	initialiseNode(*sailingLogic, "LineFollow Node", NodeImportance::CRITICAL);
	initialiseNode(*lowLevelControllerNodeASPire, "LowLevelControllerNodeJanet Node", NodeImportance::CRITICAL);
  #endif
	// Start active nodes

	#if SIMULATION == 1
	simulation.start();
	#else
	xbee.start();
	windSensor.start();
	compass.start();
	gpsd.start();
	#endif

  httpsync.start();

  #if LOCAL_NAVIGATION_MODULE == 1
  vesselState.start();
  lnm.start();
  collidableMgr.startGC();
  #else
	stateEstimationNode.start();
	sailingLogic->start();
  #endif

	// NOTE - Jordan: Just to ensure messages are following through the system
	MessagePtr dataRequest = std::make_unique<DataRequestMsg>(NodeID::MessageLogger);
	messageBus.sendMessage(std::move(dataRequest));

  // NOTE: Maybe add to db aswell?
	int dbLoggerWaitTime = 100; 		// wait time (in milliseconds) between messages from the messageBus
	int dbLoggerUpdateFrequency = 1000; // updating frequency to the database (in milliseconds)
	int dbLoggerQueueSize = 5; 			// how many messages to log to the databse at a time

	DBLoggerNode dbLoggerNode(messageBus, dbHandler, dbLoggerWaitTime, dbLoggerUpdateFrequency, dbLoggerQueueSize);
	initialiseNode(dbLoggerNode, "DBLoggerNode", NodeImportance::CRITICAL);
	dbLoggerNode.start();

	Logger::info("Message bus started!");
	messageBus.run();


	Logger::shutdown();
  #if LOCAL_NAVIGATION_MODULE == 0
	delete sailingLogic;
	delete lowLevelControllerNodeASPire;
  #endif
	exit(0);
}
