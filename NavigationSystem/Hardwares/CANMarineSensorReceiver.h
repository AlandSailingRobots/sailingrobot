/****************************************************************************************
 *
 * File:
 * 		CANMarineSensorReciver.h
 *
 * Purpose:
 *		Recive sensorvalues from CANMarineSensorTransmissionNode,
 *       calculte salinity and do a MarineSensorDataMsg
 *
 * Developer Notes:
 *
 *
 *
 ***************************************************************************************/

//	virtual void processFrame(CanMsg& msg) = 0;
// + look at CANArduinoNode for how to make the constructor (same folder)

#pragma once

#include "../Hardwares/CAN_Services/CANFrameReceiver.h"
#include "../Hardwares/CAN_Services/CANService.h"
#include "../MessageBus/MessageBus.h"

class CANMarineSensorReceiver : public CANFrameReceiver {
   public:
    CANMarineSensorReceiver(MessageBus& messageBus, CANService& canService);  // from
                                                                              // CANArduinoNode,

    virtual void processFrame(CanMsg& msg);

   private:
    MessageBus& m_msgBus;
};
