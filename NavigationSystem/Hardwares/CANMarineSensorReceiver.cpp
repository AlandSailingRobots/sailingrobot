#include <iostream>
#include "CANMarineSensorReceiver.h"
#include "CAN_Services/CANService.h"
#include "Math/Utility.h"
#include "Messages/MarineSensorDataMsg.h"
#include "SystemServices/Logger.h"
#include "../ArduinoSketches/libraries/common/canbus_id_defs.h"
#include "../ArduinoSketches/libraries/common/CanMessageHandler.h"


const int PH_INTERVAL_MIN = 0;
const int PH_INTERVAL_MAX = 14;

const int CONDUCTIVETY_INTERVAL_MIN = 5;
const int CONDUCTIVETY_INTERVAL_MAX = 200000;

const int TEMPERATURE_INTERVAL_MIN = -5;
const int TEMPERATURE_INTERVAL_MAX = 40;


CANMarineSensorReceiver::CANMarineSensorReceiver(MessageBus& messageBus, CANService& canService) :
CANFrameReceiver(canService, MSG_ID_MARINE_SENSOR_DATA), m_msgBus(messageBus)
{
}

void CANMarineSensorReceiver::processFrame (CanMsg& msg) {
    Logger::info("Recieved marine sensor readings from CanBus");

    CanMessageHandler handler(msg);

    float ph = handler.getMappedData(1,PH_INTERVAL_MIN,PH_INTERVAL_MAX);
    float conductivety = handler.getMappedData(4,CONDUCTIVETY_INTERVAL_MIN,CONDUCTIVETY_INTERVAL_MAX);
    float temp = handler.getMappedData(2,TEMPERATURE_INTERVAL_MIN,TEMPERATURE_INTERVAL_MAX);
    float salinity = Utility::calculateSalinity (temp, conductivety);

    MessagePtr marineSensorDataMsg = std::make_unique<MarineSensorDataMsg>(temp, conductivety, ph, salinity);
    m_msgBus.sendMessage(std::move(marineSensorDataMsg));



    if(handler.getError() > 0) {
        Logger::error("Error from marine sensors, error code: %d", handler.getError());
    }


}
