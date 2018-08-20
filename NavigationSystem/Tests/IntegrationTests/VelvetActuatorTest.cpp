 

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

#include "../Math/Utility.h"

 #include "../Messages/SailCommandMsg.h"
 #include "../Messages/RudderCommandMsg.h"



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
    
    int setPosition = 1470; // pwm value for sailwinch and rudder servo in middle course (sail in, rudder centered)

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

}


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
	std::string portname = "/dev/ttyUSB0";
	int channel = 4;
	int speed = 0;
	int acceleration = 0;
	VelvetActuatorTest rudder(messageBus, NodeID::RudderActuator, channel, speed, acceleration);

	MaestroController::init(portname);


	// Initialise nodes
	//-------------------------------------------------------------------------------
    
	initialiseNode(rudder, "Rudder Actuator", NodeImportance::CRITICAL);

	// Start active nodes
	//-------------------------------------------------------------------------------


	// Begins running the message bus
	//-------------------------------------------------------------------------------
	Logger::info("Message bus started!");
	messageBus.run();


	exit(0);
}

