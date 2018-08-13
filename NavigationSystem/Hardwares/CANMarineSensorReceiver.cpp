#include <iostream>
#include "CANMarineSensorReceiver.h"
#include "CAN_Services/CANService.h"
#include "../Math/Utility.h"
#include "../Messages/MarineSensorDataMsg.h"
#include "../SystemServices/Logger.h"
#include "CAN_Services/CanBusCommon/canbus_defs.h"
#include "CAN_Services/CanBusCommon/CanMessageHandler.h"



CANMarineSensorReceiver::CANMarineSensorReceiver(MessageBus& messageBus, CANService& canService) :
CANFrameReceiver(canService, MSG_ID_MARINE_SENSOR_DATA), m_msgBus(messageBus)
{
}

void CANMarineSensorReceiver::processFrame (CanMsg& msg) {
    Logger::trace("Received marine sensor readings from CanBus");

    CanMessageHandler handler(msg);

    if(handler.getMessageId() == MSG_ID_MARINE_SENSOR_DATA) {
        Float16Compressor fltCompressor;
        float ph, conductivety, temperature;
        uint16_t comp_temperature;
        handler.canMsgToBitset(); // update bitset with m_message.data
        handler.getMappedData(&ph, SENSOR_PH_START, SENSOR_PH_DATASIZE, SENSOR_PH_IN_BYTE, SENSOR_PH_INTERVAL_MIN, SENSOR_PH_INTERVAL_MAX);
        handler.getMappedData(&conductivety, SENSOR_CONDUCTIVETY_START, SENSOR_CONDUCTIVETY_DATASIZE, SENSOR_CONDUCTIVETY_IN_BYTE, SENSOR_CONDUCTIVETY_INTERVAL_MIN, SENSOR_CONDUCTIVETY_INTERVAL_MAX);

        handler.getData(&comp_temperature, SENSOR_TEMPERATURE_START, SENSOR_TEMPERATURE_DATASIZE, SENSOR_TEMPERATURE_IN_BYTE);
        temperature = fltCompressor.decompress(comp_temperature);
        float salinity = Utility::calculateSalinity (temperature, conductivety);

        MessagePtr marineSensorDataMsg = std::make_unique<MarineSensorDataMsg>(static_cast<float>(temperature), 
                                     static_cast<float>(conductivety), static_cast<float>(ph), salinity);
        m_msgBus.sendMessage(std::move(marineSensorDataMsg));


        if(handler.getErrorMessage() > 0) {
            Logger::trace("Error from marine sensors, error code: %d", handler.getErrorMessage());
        }
    }
}
