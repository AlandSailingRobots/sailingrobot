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
 #include "Hardwares/CANWindsensorNode.h"
#endif

#include "Navigation/WaypointMgrNode.h"
#include "WorldState/StateEstimationNode.h"
#include "WorldState/WindStateNode.h"
#include "HTTPSync/HTTPSyncNode.h"
#include "Xbee/XbeeSyncNode.h"
#if LOCAL_NAVIGATION_MODULE == 1
  #include "WorldState/VesselStateNode.h"
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


#if USE_OPENCV_COLOR_DETECTION == 1
#include "WorldState/obstacledetection/colorDetectionNode.h"
#endif

#include "Messages/DataRequestMsg.h"
#include "DataBase/DBHandler.h"
#include "DataBase/DBLoggerNode.h"
#include "Hardwares/MaestroController/MaestroController.h"
#include "Xbee/Xbee.h"

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
/// Used for development of the Local Navigation Module
///
///----------------------------------------------------------------------------------
#if LOCAL_NAVIGATION_MODULE==1
void development_LocalNavigationModule( MessageBus& messageBus, DBHandler& dbHandler)
{
	//const double PGAIN = 0.20;
	//const double IGAIN = 0.30;
	const int16_t MAX_VOTES = 25;

	Logger::info( "Using Local Navigation Module" );

	VesselStateNode vesselState	( messageBus, 0.2 );
	WaypointMgrNode waypoint	( messageBus, dbHandler );
	LocalNavigationModule lnm	( messageBus );
  //Node* llc;
	//llc	= new LowLevelControllerNodeJanet( messageBus, PGAIN, IGAIN, dbHandler );
	CollidableMgr collidableMgr;

	#if SIMULATION == 1
	SimulationNode 	simulation	( messageBus, &collidableMgr );
	#endif

	initialiseNode( vesselState, 	"Vessel State Node", 		NodeImportance::CRITICAL );
	initialiseNode( waypoint, 		"Waypoint Node", 			NodeImportance::CRITICAL );
	initialiseNode( lnm,			"Local Navigation Module",	NodeImportance::CRITICAL );
	//initialiseNode( *llc,			"Low Level Controller",		NodeImportance::CRITICAL );

	#if SIMULATION == 1
	initialiseNode( simulation, 	"Simulation Node", 			NodeImportance::CRITICAL );
	#endif

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


	vesselState.start();
	lnm.start();

	collidableMgr.startGC();

	#if SIMULATION == 1
	simulation.start();
	#endif

	Logger::info("Message bus started!");

	// Returns when the program has been closed (Does the program ever close gracefully?)
	messageBus.run();
}
#endif

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

	// Create nodes
	//MessageLoggerNode msgLogger(messageBus);
	//CollidableMgr collidableMgr;

  #if LOCAL_NAVIGATION_MODULE == 1
    development_LocalNavigationModule(messageBus, dbHandler);
    Logger::shutdown();
  #else

	#if SIMULATION == 1
	SimulationNode 	simulation	( messageBus ); // , &collidableMgr );
	#else

  CANService canService;
	XbeeSyncNode xbee(messageBus, dbHandler);

  CANWindsensorNode windSensor(messageBus, canService,
                                dbHandler.retrieveCellAsInt("windState_config","1","time_filter_ms"));
	HMC6343Node compass(messageBus, dbHandler.retrieveCellAsInt("buffer_config", "1", "compass"));
  GPSDNode gpsd(messageBus,
                dbHandler.retrieveCellAsDouble("gpsd_config", "1", "loop_time"));

  // NOTE: What replaces ArduinoNode, nothing?
	//ArduinoNode arduino(messageBus, 0.1);
	std::vector<std::string> list;
	list.push_back("red");
	#endif


  HTTPSyncNode httpsync(messageBus, &dbHandler,
                        dbHandler.retrieveCellAsInt("httpsync_config", "1","delay"),
                        dbHandler.retrieveCellAsInt("httpsync_config","1","remove_logs"));
  StateEstimationNode stateEstimationNode(messageBus,
                                          dbHandler.retrieveCellAsDouble("vesselState_config","1", "loop_time"),
                                          dbHandler.retrieveCellAsDouble("vesselState_config", "1", "speedLimit"));
  WindStateNode windStateNode(messageBus,
                              dbHandler.retrieveCellAsDouble("windState_config", "1", "time_filter_ms"));
	WaypointMgrNode waypoint(messageBus, dbHandler);


	ActiveNode* sailingLogic;
  Node* lowLevelControllerNodeASPire;

	sailingLogic = new LineFollowNode(messageBus, dbHandler);
  lowLevelControllerNodeASPire = new LowLevelControllerNodeASPire(messageBus, canService, 45, 30, 50, 10); // NOTE: Values to be read from db

  /* NOTE: It compiles, but is ActuatorNodeASPire really working as intended?
  * Should be uncommented once ActuatorNodeAspire class is finsihed.
  *
	#if SIMULATION == 0
	int channel = dbHandler.retrieveCellAsInt("sail_servo_config", "1", "channel");
	int speed = dbHandler.retrieveCellAsInt("sail_servo_config", "1", "speed");
	int acceleration = dbHandler.retrieveCellAsInt("sail_servo_config", "1", "acceleration");
  */
	ActuatorNodeASPire sail(messageBus, canService);

	//channel = dbHandler.retrieveCellAsInt("rudder_servo_config", "1", "channel");
	//speed = dbHandler.retrieveCellAsInt("rudder_servo_config", "1", "speed");
	//acceleration = dbHandler.retrieveCellAsInt("rudder_servo_config", "1", "acceleration");

	ActuatorNodeASPire rudder(messageBus, canService);
	//MaestroController::init(dbHandler.retrieveCell("maestro_controller_config", "1", "port"));
	//#endif
  /* */
	bool requireNetwork = (bool) (dbHandler.retrieveCellAsInt("sailing_robot_config", "1", "require_network"));

	// Initialise nodes
	//initialiseNode(msgLogger, "Message Logger", NodeImportance::NOT_CRITICAL);

	#if SIMULATION == 1
	initialiseNode(simulation,"Simulation Node",NodeImportance::CRITICAL);
	#else
	initialiseNode(xbee, "Xbee Sync Node", NodeImportance::NOT_CRITICAL);
	initialiseNode(windSensor, "Wind Sensor", NodeImportance::CRITICAL); // TODO: Uncomment this when the rest is done
	initialiseNode(compass, "Compass", NodeImportance::CRITICAL);
	initialiseNode(gpsd, "GPSD Node", NodeImportance::CRITICAL);
	//initialiseNode(sail, "Sail Actuator", NodeImportance::CRITICAL);
	//initialiseNode(rudder, "Rudder Actuator", NodeImportance::CRITICAL);
	//initialiseNode(arduino, "Arduino Node", NodeImportance::NOT_CRITICAL);
	//initialiseNode(colorDetection, "Colour detection node", NodeImportance::NOT_CRITICAL);
	#endif

	if (requireNetwork)
	{
		initialiseNode(httpsync, "Httpsync Node", NodeImportance::CRITICAL);
	}
	else
	{
		initialiseNode(httpsync, "Httpsync Node", NodeImportance::NOT_CRITICAL);
	}

	//initialiseNode(vessel, "Vessel State Node", NodeImportance::CRITICAL); // TODO: Uncomment this when the rest is done
  initialiseNode(stateEstimationNode,"StateEstimation Node",NodeImportance::CRITICAL);
	initialiseNode(windStateNode,"WindState Node",NodeImportance::CRITICAL);
	initialiseNode(waypoint, "Waypoint Node", NodeImportance::CRITICAL);
	initialiseNode(*sailingLogic, "LineFollow Node", NodeImportance::CRITICAL);
	initialiseNode(*lowLevelControllerNodeASPire, "LowLevelControllerNodeJanet Node", NodeImportance::CRITICAL);

	// Start active nodes
	#if SIMULATION == 1
	simulation.start();
	#else
	xbee.start();
	//windSensor.start();
	compass.start();
	gpsd.start();
	//arduino.start();
	#endif

	httpsync.start();
	stateEstimationNode.start();
	sailingLogic->start();

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
	delete sailingLogic;
	//delete lowLevelControllerNodeJanet;
  #endif
	exit(0);
}
