#include "Hardwares/ActuatorNodeASPire.h"

#include "Messages/ActuatorControlASPireMessage.h"

ActuatorNodeASPire::ActuatorNodeASPire(MessageBus& msgBus, CANService& CANService)
: Node(NodeID::ActuatorNodeASPire, msgBus), m_CANService(&CANService)
{
    msgBus.registerNode(*this, MessageType::ActuatorControlASPire);
}

ActuatorNodeASPire::~ActuatorNodeASPire()
{

}

bool ActuatorNodeASPire::init()
{
    return true;
}

void ActuatorNodeASPire::processMessage(const Message* message)
{
    MessageType type = message->messageType();

    if(type == MessageType::ActuatorControlASPire) {
        const ActuatorControlASPireMessage* actMsg = dynamic_cast<const ActuatorControlASPireMessage*>(message);
        double rudderAngle = actMsg->rudderAngle();
        double servoAngle = actMsg->wingsailServoAngle();
        bool WindvaneSelfSteeringOn = actMsg->windvaneSelfSteering();
        int MaxRudderAngle = 30;
        int MaxServoSailAngle = 10;

        CanMsg Cmsg;
        Cmsg.id = 700;
        Cmsg.header.ide = 0;
        Cmsg.header.length = 8;
        
        rudderAngle += MaxRudderAngle;
        double ratio = 65535 / MaxRudderAngle * 2;
        uint16_t angle_16 = rudderAngle * ratio;

        (Cmsg.data[0] = angle_16 & 0xff);
        (Cmsg.data[1] = angle_16 >> 8);

        servoAngle += MaxServoSailAngle;
        ratio = 65535 / MaxServoSailAngle * 2;
        angle_16 = servoAngle * ratio;

        (Cmsg.data[2] = angle_16 & 0xff);
        (Cmsg.data[3] = angle_16 >> 8);
        (Cmsg.data[4] = 0);
        (Cmsg.data[5] = 0);
        (Cmsg.data[6] = WindvaneSelfSteeringOn);
        (Cmsg.data[7] = 0);

        m_CANService->sendCANMessage(Cmsg);
    }

}