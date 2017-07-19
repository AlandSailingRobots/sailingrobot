/****************************************************************************************
*
* File:
* 		CANArduinoNode.cpp
*
* Purpose:
*		 Sends data to the Actuator unit with the rudder and wingsail angles over the CAN bus. 
*
* Developer Notes:
*		 The CAN id numbers for the node are:
*			700
*
***************************************************************************************/


#include "Hardwares/ActuatorNodeASPire.h"
#include "Math/Utility.h"
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
        double wingsailAngle = actMsg->wingsailServoAngle();
        bool WindvaneSelfSteeringOn = actMsg->windvaneSelfSteering();
        

        CanMsg Cmsg;
        Cmsg.id = 700;
        Cmsg.header.ide = 0;
        Cmsg.header.length = 8;
        
       
        uint16_t angle_16 = Utility::mapInterval (rudderAngle, -MAX_RUDDER_ANGLE, MAX_RUDDER_ANGLE, 0 , INT16_SIZE);

        (Cmsg.data[0] = angle_16 & 0xff);
        (Cmsg.data[1] = angle_16 >> 8);

        
        angle_16 = Utility::mapInterval (wingsailAngle, -MAX_WINGSAIL_ANGLE, MAX_WINGSAIL_ANGLE, 0 , INT16_SIZE);

        (Cmsg.data[2] = angle_16 & 0xff);
        (Cmsg.data[3] = angle_16 >> 8);
        (Cmsg.data[4] = 0);
        (Cmsg.data[5] = 0);
        (Cmsg.data[6] = WindvaneSelfSteeringOn);
        (Cmsg.data[7] = 0);

        m_CANService->sendCANMessage(Cmsg);
    }

}