 

/****************************************************************************************
 *
 * File:
 * 		ActuatorNodeVelvet.cpp
 *
 * Purpose:
 *		Controls an actuator on the vessel.
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/

#include "../Hardwares/ActuatorNodeVelvet.h"
#include "../Messages/VelvetActuatorFeedbackMsg.h"
#include "../Hardwares/MaestroController/MaestroController.h"
#include "../SystemServices/Logger.h"
 
#include <ncurses.h>
#include <unordered_map>
#include <thread>
#include <sstream>
#include "../MessageBus/MessageTypes.h"
#include "../MessageBus/MessageBus.h"
#include "../MessageBus/NodeIDs.h"
#include "../Database/DBHandler.h"

#include "../Math/Utility.h"

 #include "../Messages/SailCommandMsg.h"
 #include "../Messages/RudderCommandMsg.h"
 #include "../LowLevelControllers/SailControlNode.h"
 #include "../LowLevelControllers/CourseRegulatorNode.h"



class VelvetActuatorTest: public Node {

public:
    
	VelvetActuatorTest(MessageBus& msgBus, NodeID id, int channel, int speed, int acceleration)
        :Node(NodeID::VelvetActuatorNode, msgBus), m_msgBus(msgBus), m_Channel(channel), m_Speed(speed), m_Acceleration(acceleration)
	{
	    //msgBus.registerNode(*this,MessageType::VelvetActuatorFeedback);
	    msgBus.registerNode(*this,MessageType::ServerConfigsReceived);
	    msgBus.registerNode(*this,MessageType::RudderCommand);
	    msgBus.registerNode(*this,MessageType::SailCommand);
	}

bool init()
{
    if( MaestroController::writeCommand(MaestroCommands::SetSpeed, m_Channel, m_Speed) &&
        MaestroController::writeCommand(MaestroCommands::SetAcceleration, m_Channel, m_Acceleration) )
    {
        return true;
    }
    else
    {
        Logger::error("%s Failed to write actuator speed and acceleration!", __PRETTY_FUNCTION__);
        return false;
    }

}

	void updateConfigsFromDB()
	{
	    //m_LoopTime = m_db.retrieveCellAsDouble("config_can_arduino","1","loop_time");
	}

	void processMessage(const Message* message)
{
    
    int setPosition = 1000 + (470 + m_count)%900; //testing values
    m_count += 50;

    if(message->messageType() == MessageType::SailCommand)
    {
        SailCommandMsg* msg = (SailCommandMsg*)message;
        if (nodeID() == NodeID::SailActuator) 
        {
            setPosition = msg->maxSailAngle(); //have to check why it is named maxSailAngle
            // and modify it for smartwinch
        }
    }

    if(message->messageType() == MessageType::RudderCommand)
    {
    	RudderCommandMsg* msg = (RudderCommandMsg*)message;
    	if (nodeID() == NodeID::RudderActuator) 
        {
            setPosition = msg->rudderAngle(); //have ot modify it for velvet's rudder servo
        }
    }

    if(not MaestroController::writeCommand(MaestroCommands::SetPosition, m_Channel, setPosition))
	{
		Logger::error("%s Actuator: %d Failed to write position command", __PRETTY_FUNCTION__, (int)nodeID());
	}
	Logger::info("Current command: %d", setPosition);
	usleep(1000000); //sleep 1 sec before each new command for testing

}

	int m_count; //used for testing different pwm values
private:
	    MessageBus& m_msgBus;
	    double m_LoopTime;
	    std::mutex m_lock;
	    int m_Channel;
	    int m_Speed;
	    int m_Acceleration;
	    

	};


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
	printf("\t\t\t\tVelvet Actuator Sensor Integration Test\n");
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

	MessageBus messageBus;
	DBHandler dbHandler(db_path);

	// Initialise logger
	if (Logger::init())
	{
		Logger::info("Built on %s at %s", __DATE__, __TIME__);
    	Logger::info("Velvet Actuator Integration Test");
		Logger::info("Logger init\t\t[OK]");
	}
	else
	{
		Logger::error("Logger init\t\t[FAILED]");
	}

	// Declare nodes
	//-------------------------------------------------------------------------------
	std::string portname = "/dev/ttyACM0";
	int channel = 3;
	int speed = 0;
	int acceleration = 0;
	VelvetActuatorTest rudder(messageBus, NodeID::RudderActuator, channel, speed, acceleration);

	rudder.m_count = 0;

	MaestroController::init(portname);

	SailControlNode sailControlNode(messageBus, dbHandler);
	CourseRegulatorNode courseRegulatorNode(messageBus, dbHandler);

	// Initialise nodes
	//-------------------------------------------------------------------------------
    //initialiseNode(sailControlNode, "Sail Controller", NodeImportance::CRITICAL);
    //initialiseNode(courseRegulatorNode, "Course Regulator", NodeImportance::CRITICAL);
	initialiseNode(rudder, "Rudder Actuator", NodeImportance::CRITICAL);

	// Start active nodes
	//-------------------------------------------------------------------------------

	
	// Begins running the message bus
	//-------------------------------------------------------------------------------
	Logger::info("Message bus started!");
	//messageBus.run();

//    int getPosition = 0;
	while(rudder.m_count < 10000000) {

		int setPosition = 6000 + ( rudder.m_count)%2000; //testing values
	    rudder.m_count += 100;

	    if(not MaestroController::writeCommand(MaestroCommands::SetPosition, channel, setPosition))
		{
			Logger::error("%s Actuator: %d Failed to write position command", __PRETTY_FUNCTION__, 1);
		}
		Logger::info("Current command: %d", setPosition);


		usleep(1000000); //sleep 0.5 sec before each new command for testing

	}

	exit(0);
}

