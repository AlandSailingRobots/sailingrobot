
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

#define DISABLE_LOGGING 0

enum class NodeImportance {
	CRITICAL,
	NOT_CRITICAL
};

#if TARGET == 0
#define TARGET_STR "Default"

#include "Nodes/WaypointMgrNode.h"
#include "Nodes/VesselStateNode.h"
#include "Nodes/HTTPSyncNode.h"
#include "Nodes/XbeeSyncNode.h"
#include "Nodes/RoutingNode.h"
#include "Nodes/LineFollowNode.h"
#include "Messages/DataRequestMsg.h"
#include "dbhandler/DBHandler.h"
#include "SystemServices/MaestroController.h"

#elif TARGET == 1
#define TARGET_STR "WRSC2016"

#include "Nodes/UDPNode.h"
#include "Nodes/GPSDNode.h"
#include "Nodes/SerialGPSNode.h"
#include "Nodes/MA3WindSensorNode.h"
#include "Nodes/RazorCompassNode.h"

#include "Nodes/WaypointMgrNode.h"
#include "Nodes/VesselStateNode.h"
#include "Nodes/RoutingNode.h"
#include "Nodes/LineFollowNode.h"
#include "dbhandler/DBHandler.h"
#include "SystemServices/MaestroController.h"

#define BAK_STRAT 0
#if BAK_STRAT == 1
 #include "Nodes/CollAvoidanceBakStrat.h"
#else
 #include "Nodes/CollisionAvoidanceNode.h"
#endif

#include "Messages/ActuatorPositionMsg.h"

RazorCompassNode* razorFix;

#elif TARGET == 2
#define TARGET_STR "MANCONTROL"

#include "Nodes/UDPNode.h"
#include "Nodes/GPSDNode.h"
#include "Nodes/MA3WindSensorNode.h"
#include "Nodes/RazorCompassNode.h"
#include "Nodes/ManualControlNode.h"

#include "Nodes/WaypointMgrNode.h"
#include "Nodes/VesselStateNode.h"
#include "Nodes/RoutingNode.h"
#include "Nodes/LineFollowNode.h"
#include "dbhandler/DBHandler.h"
#include "SystemServices/MaestroController.h"

RazorCompassNode* razorFix;

#else
#define TARGET_STR "None"
#endif


#include <signal.h>
#include <atomic>
#include <cstring>

#include "WRSC.h"

void got_signal(int)
{
#if TARGET == 1
	razorFix->shutdown();
#endif
	exit(0);
}


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

void setupDB(DBHandler& dbHandler)
{
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
}

///----------------------------------------------------------------------------------
/// Entry point, can accept one argument containing a relative path to the database.
///
///----------------------------------------------------------------------------------
int main(int argc, char *argv[])
{
	// This is for eclipse development so the output is constantly pumped out.
	setbuf(stdout, NULL);

	struct sigaction sa;
	    memset( &sa, 0, sizeof(sa) );
	    sa.sa_handler = got_signal;
	    sigfillset(&sa.sa_mask);
	    sigaction(SIGINT,&sa,NULL);

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
		Logger::info("Target: %s", TARGET_STR);
		Logger::info("\nLogger init\t\t[OK]");
	}
	else {
		Logger::error("Logger init\t\t[FAILED]");
	}

	MessageBus messageBus;
	DBHandler dbHandler(db_path);

	MessageLoggerNode msgLogger(messageBus);
	setupDB(dbHandler);

	// All active nodes will have a reference here so we can start them after ever node has been initialised
	std::vector<ActiveNode*> activeNodes;

	//---------------------------------------------------------------------------------------------
	// Use Simulator

#if SIMULATION == 1
	Logger::info("Using the simulator!");
	SimulationNode simulation(messageBus);
	initialiseNode(simulation, "Simulation Node", NodeImportance::CRITICAL);
	activeNodes.push_back(&simulation);
#endif

	//---------------------------------------------------------------------------------------------

	//---------------------------------------------------------------------------------------------
	// Target: Default
#if TARGET == 0

	// No sensor nodes if we are using the simulator
#if SIMULATION != 1
	// Common Sensor Nodes
	CV7Node windSensor(messageBus, dbHandler.retrieveCell("windsensor_config", "1", "port"), dbHandler.retrieveCellAsInt("windsensor_config", "1", "baud_rate"));
	HMC6343Node compass(messageBus, dbHandler.retrieveCellAsInt("buffer_config", "1", "compass"));
	GPSDNode gpsd(messageBus);
	ArduinoNode arduino(messageBus);

	activeNodes.push_back(&windSensor);
	activeNodes.push_back(&compass);
	activeNodes.push_back(&gpsd);
	activeNodes.push_back(&arduino);

	initialiseNode(windSensor, "Wind Sensor", NodeImportance::CRITICAL);
	initialiseNode(compass, "Compass", NodeImportance::CRITICAL);
	initialiseNode(gpsd, "GPSD Node", NodeImportance::CRITICAL);
	initialiseNode(arduino, "Arduino Node", NodeImportance::NOT_CRITICAL);
#endif

	// Network Nodes
	XbeeSyncNode xbee(messageBus, dbHandler);
	HTTPSyncNode httpsync(messageBus, &dbHandler, 0, false);

	activeNodes.push_back(&xbee);
	activeNodes.push_back(&httpsync);

	// Sailing Logic nodes
	VesselStateNode vessel(messageBus);
	WaypointMgrNode waypoint(messageBus, dbHandler);

	activeNodes.push_back(&vessel);

	Node* sailingLogic;
	bool usingLineFollow = (bool)(dbHandler.retrieveCellAsInt("sailing_robot_config", "1", "line_follow"));
	if(usingLineFollow)
	{
		sailingLogic = new LineFollowNode(messageBus, dbHandler);
	}
	else
	{
		sailingLogic = new RoutingNode(messageBus, dbHandler);
	}

	// Actuator Node

	int channel = dbHandler.retrieveCellAsInt("sail_servo_config", "1", "channel");
	int speed = dbHandler.retrieveCellAsInt("sail_servo_config", "1", "speed");
	int acceleration = dbHandler.retrieveCellAsInt("sail_servo_config", "1", "acceleration");

	ActuatorNode sail(messageBus, NodeID::SailActuator, channel, speed, acceleration);

	channel = dbHandler.retrieveCellAsInt("rudder_servo_config", "1", "channel");
	speed = dbHandler.retrieveCellAsInt("rudder_servo_config", "1", "speed");
	acceleration = dbHandler.retrieveCellAsInt("rudder_servo_config", "1", "acceleration");

	ActuatorNode rudder(messageBus, NodeID::RudderActuator, channel, speed, acceleration);
	MaestroController::init("dev/tty/ACM0");

	initialiseNode(xbee, "Xbee Sync Node", NodeImportance::NOT_CRITICAL);
	initialiseNode(httpsync, "Httpsync Node", NodeImportance::CRITICAL);

	initialiseNode(vessel, "Vessel State Node", NodeImportance::CRITICAL);
	initialiseNode(waypoint, "Waypoint Node", NodeImportance::CRITICAL);

	if(usingLineFollow)
	{
		initialiseNode(*sailingLogic, "LineFollow Node", NodeImportance::CRITICAL);
	}
	else
	{
		initialiseNode(*sailingLogic, "Routing Node", NodeImportance::CRITICAL);
	}


    initialiseNode(sail, "Sail Actuator", NodeImportance::CRITICAL);
	initialiseNode(rudder, "Rudder Actuator", NodeImportance::CRITICAL);
#endif
	//---------------------------------------------------------------------------------------------
    // Target: WRSC2016
    //---------------------------------------------------------------------------------------------
#if TARGET == 1

	UDPNode udp(messageBus, "172.20.26.191", 4320);
#if SIMULATION != 1

    MaestroController::init("/dev/ttyACM0");
    MA3WindSensorNode windSensor(messageBus, 11);

#if BOAT_TYPE == BOAT_ENSTA_GRAND
	GPSDNode gps(messageBus);
	RazorCompassNode compass(messageBus, "/dev/ttyUSB1");
	ActuatorNode sail(messageBus, NodeID::SailActuator, 1, 0, 0);
	ActuatorNode rudder(messageBus, NodeID::RudderActuator, 2, 0, 0);
#elif BOAT_TYPE == BOAT_ENSTA_PETIT
	SerialGPSNode gps(messageBus);
	RazorCompassNode compass(messageBus,"/dev/ttyACM1");
	ActuatorNode sail(messageBus, NodeID::SailActuator, 1, 0, 0);
	ActuatorNode rudder(messageBus, NodeID::RudderActuator, 0, 0, 0);
#endif

	razorFix = &compass;

	activeNodes.push_back(&windSensor);
	activeNodes.push_back(&gps);
	activeNodes.push_back(&compass);

    initialiseNode(compass, "Compass Node", NodeImportance::CRITICAL);
    initialiseNode(windSensor, "Wind Sensor Node", NodeImportance::CRITICAL);
    initialiseNode(gps, "GPS Node", NodeImportance::CRITICAL);

    initialiseNode(sail, "Sail Actuator", NodeImportance::CRITICAL);
    initialiseNode(rudder, "Rudder Actuator", NodeImportance::CRITICAL);

#endif
	// Sailing Logic nodes
	VesselStateNode vessel(messageBus);
	WaypointMgrNode waypoint(messageBus, dbHandler);
    CollisionAvoidanceNode collisionAvoidanceNode(messageBus);


    activeNodes.push_back(&vessel);

    Node* sailingLogic;
    bool usingLineFollow = true; //(bool)(dbHandler.retrieveCellAsInt("sailing_robot_config", "1", "line_follow"));
    if(usingLineFollow)
	{
        sailingLogic = new LineFollowNode(messageBus, dbHandler);
    }
	else
	{
        sailingLogic = new RoutingNode(messageBus, dbHandler);
    }

    // Actuator Node

    initialiseNode(collisionAvoidanceNode, "Collision Avoidance", NodeImportance::NOT_CRITICAL);
    initialiseNode(udp, "UDP Node", NodeImportance::CRITICAL);

	initialiseNode(vessel, "Vessel State Node", NodeImportance::CRITICAL);
	initialiseNode(waypoint, "Waypoint Node", NodeImportance::CRITICAL);

	initialiseNode(*sailingLogic, "Sailing Logic", NodeImportance::CRITICAL);
#endif
	//---------------------------------------------------------------------------------------------

    //---------------------------------------------------------------------------------------------
    // Target: MANCONTROL
#if TARGET == 2

    MaestroController::init("/dev/ttyACM0");

    ManualControlNode manualControl(messageBus);
    activeNodes.push_back(&manualControl);

    initialiseNode(manualControl, "Manual Control", NodeImportance::CRITICAL);

#endif
    //---------------------------------------------------------------------------------------------

	// Initialise nodes
	initialiseNode(msgLogger, "Message Logger", NodeImportance::NOT_CRITICAL);

	for(unsigned int i = 0; i < activeNodes.size(); i++)
	{
		activeNodes[i]->start();
	}

	Logger::info("Message bus started!");

	// Test actuator Positions
	// Rudder and Sail Max
	//MessagePtr actuatorMsg = std::make_unique<ActuatorPositionMsg>(RUDDER_MAX_US, SAIL_MAX_US);
	//messageBus.sendMessage(std::move(actuatorMsg));

	// Middle
	//MessagePtr actuatorMsg = std::make_unique<ActuatorPositionMsg>(RUDDER_MID_US, 1500);
	//messageBus.sendMessage(std::move(actuatorMsg));

	// Min
	MessagePtr actuatorMsg = std::make_unique<ActuatorPositionMsg>(RUDDER_MIN_US, SAIL_MIN_US);
	messageBus.sendMessage(std::move(actuatorMsg));

	messageBus.run();

	Logger::shutdown();
//	delete sailingLogic;
	exit(0);
}