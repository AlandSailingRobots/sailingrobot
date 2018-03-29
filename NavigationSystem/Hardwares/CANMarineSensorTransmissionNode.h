//
// Created by dkarlsso on 3/29/18.
//

#ifndef SAILINGROBOT_CANMARINESENSORTRANSMISSIONNODE_H
#define SAILINGROBOT_CANMARINESENSORTRANSMISSIONNODE_H


#include "../MessageBus/ActiveNode.h"
#include "../DataBase/DBHandler.h"
#include "CAN_Services/CANService.h"
#include "../MessageBus/Message.h"

class CANMarineSensorTransmissionNode : public ActiveNode {
public:
    CANMarineSensorTransmissionNode(MessageBus& msgBus, DBHandler& dbhandler, CANService& canService);
    ~CANMarineSensorTransmissionNode();

    void processMessage(const Message *message);

    void start();

private:
    static void CANSensorNodeThreadFunc(ActiveNode* nodePtr);

    CANService& m_CANService;
    std::mutex m_lock;
    double m_LoopTime;
};


#endif //SAILINGROBOT_CANMARINESENSORTRANSMISSIONNODE_H
