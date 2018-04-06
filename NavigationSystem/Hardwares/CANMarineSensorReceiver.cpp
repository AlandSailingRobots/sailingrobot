#include <iostream>
#include "CANDatalistener.h"
#include "CAN_Services/CANService.h"
#include "Math/Utility.h"
#include "Messages/MarineSensorDataMsg.h"

const float MAX_TEMP = 40;
const float MIN_TEMP = -5;
const float MAX_CON = 200000;
const float MIN_CON = 5;
const float MAX_PH = 14;
const float MIN_PH = 0;

const float ARD_MAX_TEMP = 65535;
const float ARD_MIN_TEMP = 0;
const float ARD_MAX_CON = 4294967295;
const float ARD_MIN_CON = 0;
const float ARD_MAX_PH = 65535;
const float ARD_MIN_PH = 0;


CANDatalistener::CANDatalistener(MessageBus& messageBus, CANService& canService) :
CANFrameReceiver(canService, 711), m_msgBus(messageBus)
{
}

//+ calculate salinity from mesaurments
//+ add a MarineSensorDataMsg in the same way as MarineSensorNode
void CANDatalistener::processFrame (CanMsg& msg) {

    //provided that msg frameID is 711 nothing else
    uint16_t rawPh = (msg.data[1] << 8 | msg.data[0]);
    float   ph  = Utility::mapInterval (rawPh,ARD_MIN_PH,ARD_MAX_PH,MIN_PH,MAX_PH); //extract from CanMsg and convert

    uint16_t rawCon = (msg.data[5] << 24 | msg.data[4] << 16 | msg.data[3] << 8 | msg.data[2] );
    float	conductivety = Utility::mapInterval (rawCon,ARD_MIN_CON,ARD_MAX_CON,MIN_CON,MAX_CON); //extract from CanMsg and convert

    uint16_t rawTemp = (msg.data[7] << 8 | msg.data[6]);
    float	temp = Utility::mapInterval (rawTemp,ARD_MIN_TEMP,ARD_MIN_TEMP,MIN_TEMP,MAX_TEMP); //extract from CanMsg and convert

    float 	salinity = Utility::calculateSalinity (temp, conductivety);
    MessagePtr marineSensorDataMsg = std::make_unique<MarineSensorDataMsg>(temp, conductivety, ph, salinity);
    m_msgBus.sendMessage(std::move(marineSensorDataMsg));
}
