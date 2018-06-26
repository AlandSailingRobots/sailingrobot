#include <ncurses.h>
#include <unordered_map>
#include <thread>
#include <sstream>

#include "Hardwares/CAN_Services/CANService.h"
#include "Hardwares/CAN_Services/N2kMsg.h"
#include "Hardwares/CAN_Services/CanBusCommon/canbus_defs.h"
#include "Hardwares/CAN_Services/CanBusCommon/CanMessageHandler.h"
#include "Hardwares/CAN_Services/CANService.h"
#include "Hardwares/GPSDNode.h"
#include "Hardwares/CANCurrentSensorNode.h"

#include "SystemServices/Logger.h"

#include "MessageBus/MessageTypes.h"
#include "MessageBus/MessageBus.h"
#include "MessageBus/NodeIDs.h"

#include "Messages/CurrentSensorDataMsg.h"

#include "Math/Utility.h"



class CurrentSensorReceiver : public CANFrameReceiver {

public:
    CurrentSensorReceiver(MessageBus& messageBus, CANService& canService) :
            CANFrameReceiver(canService, MSG_ID_CURRENT_SENSOR_DATA), m_msgBus(messageBus)
    {
    }

    void processFrame (CanMsg& msg) {
	        Logger::info("Received marine sensor readings from CanBus");
	    Float16Compressor fltCompressor;
		CanMessageHandler messageHandler(msg);
		float current, voltage;
		SensedElement element;
		uint16_t comp_current, comp_voltage;

		if (messageHandler.getMessageId() == MSG_ID_CURRENT_SENSOR_DATA) {
	                // Use get data instead(int)? Parse data here or add the routine in another file?
			messageHandler.getData(&comp_current, CURRENT_SENSOR_CURRENT_DATASIZE);
			messageHandler.getData(&comp_voltage, CURRENT_SENSOR_VOLTAGE_DATASIZE);

		}
	    current = fltCompressor.decompress(comp_current);
	    voltage = fltCompressor.decompress(comp_voltage);
	    element = SAILDRIVE;                                                 // TO CHANGE FOR MULTI SENSOR READING
	    MessagePtr currentSensorDataMsg = std::make_unique<CurrentSensorDataMsg>(static_cast<float>(current),
	                                                static_cast<float>(voltage), static_cast<SensedElement>(element));
	    m_msgBus.sendMessage(std::move(currentSensorDataMsg));
	    Logger::info(" Current sensor data: \n Current: %lf \n Voltage: %lf \n",current,voltage);
	}

private:
    MessageBus& m_msgBus;

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
	printf("\t\t\t\tSailing Robot Current Sensor Integration Test\n");
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
    	Logger::info("Current Sensor Integration Test");
		Logger::info("Logger init\t\t[OK]");
	}
	else
	{
		Logger::error("Logger init\t\t[FAILED]");
	}

	// Declare nodes
	//-------------------------------------------------------------------------------

	CANService canService;

    CurrentSensorReceiver canCurrentSensorReceiver(messageBus, canService);
	//CANMarineSensorTransmissionNode canMarineSensorTransmissionNode(messageBus, canService);


	// Initialise nodes
	//-------------------------------------------------------------------------------
    //initialiseNode(canMarineSensorTransmissionNode, "Marine Sensors", NodeImportance::NOT_CRITICAL);


	// Start active nodes
	//-------------------------------------------------------------------------------

    auto future = canService.start();

    /*
     * Send a data request message.
     * Response is logged as info
     */
    //MessagePtr msg = std::make_unique<DataRequestMsg>();
    //messageBus.sendMessage(std::move(msg));


	// Begins running the message bus
	//-------------------------------------------------------------------------------
	Logger::info("Message bus started!");
	messageBus.run();


	//Logger::shutdown();
	exit(0);
}

