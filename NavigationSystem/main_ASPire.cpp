#include <string>
#include "../Database/DBHandler.h"
#include "../Database/DBLoggerNode.h"
#include "../HTTPSync/HTTPSyncNode.h"
#include "../MessageBus/MessageBus.h"
#include "../Messages/DataRequestMsg.h"
#include "../SystemServices/Logger.h"

#include "../WorldState/CameraProcessingUtility.h"
#include "../WorldState/AISProcessing.h"
#include "../Navigation/WaypointMgrNode.h"
#include "../WorldState/StateEstimationNode.h"
#include "../WorldState/WindStateNode.h"
#include "../WorldState/CollidableMgr/CollidableMgr.h"

#include "../LowLevelControllers/WingsailControlNode.h"
#include "../LowLevelControllers/CourseRegulatorNode.h"

#if LOCAL_NAVIGATION_MODULE == 1
  #include "../Navigation/LocalNavigationModule/LocalNavigationModule.h"
  #include "../Navigation/LocalNavigationModule/Voters/CourseVoter.h"
  #include "../Navigation/LocalNavigationModule/Voters/WaypointVoter.h"
  #include "../Navigation/LocalNavigationModule/Voters/WindVoter.h"
  #include "../Navigation/LocalNavigationModule/Voters/ChannelVoter.h"
  #include "../Navigation/LocalNavigationModule/Voters/ProximityVoter.h"
  #include "../Navigation/LocalNavigationModule/Voters/MidRangeVoter.h"
  #include "../Navigation/LocalNavigationModule/VoterTCPDebugger.h"
#else
  #include "../Navigation/LineFollowNode.h"
#endif

#if SIMULATION == 1
  #include "../Simulation/SimulationNode.h"
#else
  #include "../Hardwares/HMC6343Node.h"
  #include "../Hardwares/GPSDNode.h"
  #include "../Hardwares/CAN_Services/CANService.h"
  #include "../Hardwares/CANWindsensorNode.h"
  #include "../Hardwares/ActuatorNodeASPire.h"
  #include "../Hardwares/CANArduinoNode.h"

#include "../Hardwares/CANMarineSensorReceiver.h"
#include "../Hardwares/CANMarineSensorTransmissionNode.h"

#include "../Hardwares/CANCurrentSensorNode.h"
#include "../WorldState/PowerTrackNode.h"

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
			//Logger::shutdown();
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
		//Logger::shutdown();
		exit(1);
	}


	// Declare nodes
	//-------------------------------------------------------------------------------

	int dbLoggerQueueItems = 5; 			// how many messages to log to the database at a time
	DBLoggerNode dbLoggerNode(messageBus, dbHandler, dbLoggerQueueItems);
	HTTPSyncNode httpsync(messageBus, &dbHandler);

	StateEstimationNode stateEstimationNode(messageBus, dbHandler);
	WindStateNode windStateNode(messageBus);
	WaypointMgrNode waypoint(messageBus, dbHandler);
	CollidableMgr collidableMgr;

	WingsailControlNode wingSailControlNode(messageBus, dbHandler);
	CourseRegulatorNode courseRegulatorNode(messageBus, dbHandler);

	CameraProcessingUtility cameraProcessingUtility(messageBus, dbHandler, &collidableMgr);
	AISProcessing aisProcessing(messageBus, dbHandler, &collidableMgr);

	#if LOCAL_NAVIGATION_MODULE == 1
		LocalNavigationModule lnm	( messageBus, dbHandler );

		int MAX_VOTES;
		dbHandler.getConfigFrom(MAX_VOTES, "max_vote", "config_voter_system");

		double weight = 0;
		dbHandler.getConfigFrom(weight, "course_voter_weight","config_voter_system");
		CourseVoter courseVoter(MAX_VOTES, weight);

		weight = 0;
		dbHandler.getConfigFrom(weight, "waypoint_voter_weight", "config_voter_system");
		WaypointVoter waypointVoter(MAX_VOTES, weight); // weight = 1

		weight = 0;
		dbHandler.getConfigFrom(weight, "wind_voter_weight", "config_voter_system");
		WindVoter windVoter(MAX_VOTES, weight); // weight = 1

		weight = 0;
		dbHandler.getConfigFrom(weight, "channel_voter_weight", "config_voter_system");
		ChannelVoter channelVoter( MAX_VOTES, weight); // weight = 1

		weight = 0;
		dbHandler.getConfigFrom(weight, "midrange_voter_weight", "config_voter_system");
		MidRangeVoter midRangeVoter( MAX_VOTES, weight, collidableMgr );

		weight = 0;
		dbHandler.getConfigFrom(weight, "proximity_voter_weight", "config_voter_system");
		ProximityVoter proximityVoter( MAX_VOTES, weight, collidableMgr);



		lnm.registerVoter( &courseVoter );
		lnm.registerVoter( &waypointVoter );
		lnm.registerVoter( &windVoter );
		lnm.registerVoter( &channelVoter );

		// As there is no veto used in both avoidance voters for now, you can disable avoidance system just by
		// putting a weigth of zero in config_ASPire.json and push the new config manually or through the website.
		//lnm.registerVoter( &proximityVoter );
		//lnm.registerVoter( &midRangeVoter );

        //VoterTCPDebugger voterTCPD(messageBus, lnm, 3);
        //VoterTCPDebugger voterTCPD(messageBus, courseVoter);

    #else
		LineFollowNode sailingLogic(messageBus, dbHandler);
  	#endif


	#if SIMULATION == 1
  		SimulationNode simulation(messageBus, 1, &collidableMgr);
  	#else
		CANService canService;

		HMC6343Node compass(messageBus, dbHandler);
	  	GPSDNode gpsd(messageBus, dbHandler);
		CANWindsensorNode windSensor(messageBus, dbHandler, canService);
	  	ActuatorNodeASPire actuators(messageBus, canService);
	  	CANArduinoNode actuatorFeedback(messageBus, dbHandler, canService);



		CANMarineSensorReceiver canMarineSensorReciver(messageBus, canService);

		CANMarineSensorTransmissionNode canMarineSensorTransmissionNode(messageBus, canService);
		CANCurrentSensorNode canCurrentSensorNode(messageBus, dbHandler, canService);
		PowerTrackNode powerTrackNode(messageBus, dbHandler, 0.5);


	#endif


	// Initialise nodes
	//-------------------------------------------------------------------------------

	initialiseNode(httpsync, "Httpsync", NodeImportance::NOT_CRITICAL); // This node is not critical during the developement phase.
	initialiseNode(dbLoggerNode, "DBLogger", NodeImportance::CRITICAL);

	initialiseNode(stateEstimationNode,"StateEstimation",NodeImportance::CRITICAL);
	initialiseNode(windStateNode,"WindState",NodeImportance::CRITICAL);
	initialiseNode(waypoint, "Waypoint", NodeImportance::CRITICAL);

 	initialiseNode(wingSailControlNode, "Wing Sail Controller", NodeImportance::CRITICAL);
 	initialiseNode(courseRegulatorNode, "Course Regulator", NodeImportance::CRITICAL);

	#if LOCAL_NAVIGATION_MODULE == 1
		initialiseNode( lnm, "Local Navigation Module",	NodeImportance::CRITICAL );
		//initialiseNode( voterTCPD, "VoterTCPDebugger", NodeImportance::NOT_CRITICAL);
	#else
		initialiseNode(sailingLogic, "LineFollow", NodeImportance::CRITICAL);
	#endif


	#if SIMULATION == 1
		initialiseNode(simulation,"Simulation",NodeImportance::CRITICAL);
	#else
		initialiseNode(compass, "Compass", NodeImportance::CRITICAL);
		initialiseNode(gpsd, "GPSD", NodeImportance::CRITICAL);
		initialiseNode(windSensor, "Wind Sensor", NodeImportance::CRITICAL);
		initialiseNode(actuators, "Actuators", NodeImportance::CRITICAL);
		initialiseNode(actuatorFeedback, "Actuator Feedback", NodeImportance::NOT_CRITICAL);
		initialiseNode(canMarineSensorTransmissionNode, "Marine Sensors", NodeImportance::NOT_CRITICAL);
		initialiseNode(canCurrentSensorNode, "Current Sensors", NodeImportance::NOT_CRITICAL);
		initialiseNode(powerTrackNode, "Powertrack", NodeImportance::NOT_CRITICAL);
	#endif

	initialiseNode(cameraProcessingUtility, "Camera Processing", NodeImportance::NOT_CRITICAL);

	// Start active nodes
	//-------------------------------------------------------------------------------

	httpsync.start();
	dbLoggerNode.start();

	stateEstimationNode.start();
	collidableMgr.startGC();

	wingSailControlNode.start();
	courseRegulatorNode.start();



	#if SIMULATION == 1
		simulation.start();
	#else
	  	auto future = canService.start();
		compass.start();
		gpsd.start();
		windSensor.start();
		actuatorFeedback.start();
		canCurrentSensorNode.start();
		powerTrackNode.start();
	#endif

	#if LOCAL_NAVIGATION_MODULE == 1
    // Cancel Camera Processing node if using voters without avoidance.
    // Camera processing enabled for voter system only currently.
	//  cameraProcessingUtility.start();
		lnm.start();
		//voterTCPD.start();
	#else
		sailingLogic.start();
	#endif

	// Begins running the message bus
	//-------------------------------------------------------------------------------
	Logger::info("Message bus started!");
	messageBus.run();



	//Logger::shutdown();

	exit(0);
}
