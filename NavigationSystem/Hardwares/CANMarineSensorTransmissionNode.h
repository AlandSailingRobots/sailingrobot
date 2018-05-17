/****************************************************************************************
 *
 * File:
 * 		CANMarineSensorTransmissionNode.h
 *
 * Purpose:
 *		 Reads DataRequestMsg, DataCollectionStartMsg and DataCollectionStopMsg and send reading
 *requests on the CAN Bus to the Arduino along with properties. IE. Informs Ardunio to read Marine
 *Sensors and if they should be read on a continous interval.
 *
 * Developer Notes:
 *
 *
 *
 ***************************************************************************************/

#pragma once

#include "Hardwares/CAN_Services/CANService.h"
#include "Hardwares/CAN_Services/N2kMsg.h"
#include "MessageBus/Message.h"
#include "MessageBus/MessageBus.h"
#include "MessageBus/Node.h"

class CANMarineSensorTransmissionNode : public Node {
   public:
    CANMarineSensorTransmissionNode(MessageBus& msgBus, CANService& canService);
    ~CANMarineSensorTransmissionNode();

    bool init();
    void processMessage(const Message* message);

   private:
    CANService& m_CANService;
    std::mutex m_lock;
    int m_arduinoSensorLoopTime;
    bool m_takeContinousSensorReadings;
};
