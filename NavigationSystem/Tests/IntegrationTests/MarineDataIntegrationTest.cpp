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
#include "Hardwares/CANMarineSensorTransmissionNode.h"

#include "SystemServices/Logger.h"

#include "MessageBus/MessageTypes.h"
#include "MessageBus/MessageBus.h"
#include "MessageBus/NodeIDs.h"

#include "Messages/DataRequestMsg.h"
#include "Messages/MarineSensorDataMsg.h"

#include "Math/Utility.h"


class MarineSensorReceiver : public CANFrameReceiver {

public:
    MarineSensorReceiver(MessageBus& messageBus, CANService& canService) :
            CANFrameReceiver(canService, MSG_ID_MARINE_SENSOR_DATA), m_msgBus(messageBus)
    {
    }

    void processFrame (CanMsg& msg) {
        Logger::info("Recieved marine sensor readings from CanBus");

        CanMessageHandler handler(msg);

        if(handler.getMessageId() == MSG_ID_MARINE_SENSOR_DATA) {
            double ph;
			double conductivety;
			double temp;
            handler.getMappedData(&ph, SENSOR_PH_DATASIZE,
                                              SENSOR_PH_INTERVAL_MIN, SENSOR_PH_INTERVAL_MAX);

            handler.getMappedData(&conductivety, SENSOR_CONDUCTIVETY_DATASIZE,
                                                        SENSOR_CONDUCTIVETY_INTERVAL_MIN, SENSOR_CONDUCTIVETY_INTERVAL_MAX);

            handler.getMappedData(&temp, SENSOR_TEMPERATURE_DATASIZE,
                                                SENSOR_TEMPERATURE_INTERVAL_MIN, SENSOR_TEMPERATURE_INTERVAL_MAX);
            float salinity = Utility::calculateSalinity (temp, conductivety);

            MessagePtr marineSensorDataMsg = std::make_unique<MarineSensorDataMsg>(static_cast<float>(temp), static_cast<float>(conductivety), static_cast<float>(ph), salinity);
            m_msgBus.sendMessage(std::move(marineSensorDataMsg));


            Logger::info(" Marine sensor data: \n PH: %lf \n Conductivety: %lf \n Temperature: %lf \n Error ID: %d",ph,conductivety,temp,handler.getErrorMessage());


            if(handler.getErrorMessage() > 0) {
                Logger::error("Error from marine sensors, error code: %d", handler.getErrorMessage());
            }
        }
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
	printf("\t\t\t\tSailing Robot Marine Sensor Integration Test\n");
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
    	Logger::info("Marine Sensor Integration Test");
		Logger::info("Logger init\t\t[OK]");
	}
	else
	{
		Logger::error("Logger init\t\t[FAILED]");
	}

	// Declare nodes
	//-------------------------------------------------------------------------------

	CANService canService;

    MarineSensorReceiver canMarineSensorReceiver(messageBus, canService);
	CANMarineSensorTransmissionNode canMarineSensorTransmissionNode(messageBus, canService);


	// Initialise nodes
	//-------------------------------------------------------------------------------
    initialiseNode(canMarineSensorTransmissionNode, "Marine Sensors", NodeImportance::NOT_CRITICAL);


	// Start active nodes
	//-------------------------------------------------------------------------------

    auto future = canService.start();

    /*
     * Send a data request message.
     * Response is logged as info
     */
    MessagePtr msg = std::make_unique<DataRequestMsg>();
    messageBus.sendMessage(std::move(msg));


	// Begins running the message bus
	//-------------------------------------------------------------------------------
	Logger::info("Message bus started!");
	messageBus.run();


	//Logger::shutdown();
	exit(0);
}
