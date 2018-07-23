#ifndef SAILINGROBOT_ACTUATORNODEVELVET_H
#define SAILINGROBOT_ACTUATORNODEVELVET_H

/****************************************************************************************
 *
 * File:
 * 		ActuatorNodeVelvet.h
 *
 * Purpose:
 *		Controls an actuator on the vessel.
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/


#include "../Database/DBHandler.h"
#include "../MessageBus/Node.h"

class ActuatorNodeVelvet : public Node {
public:
    ActuatorNodeVelvet(MessageBus& msgBus, NodeID id, int channel, int speed, int acceleration);

    ///----------------------------------------------------------------------------------
    /// Setups the actuator.
    ///
    ///----------------------------------------------------------------------------------
    virtual bool init();

    ///----------------------------------------------------------------------------------
    /// Processes ActuatorPositionMsgs
    ///
    ///----------------------------------------------------------------------------------
    virtual void processMessage(const Message* message);

private:
    int m_Channel;
    int m_Speed;
    int m_Acceleration;
};



#endif //SAILINGROBOT_ACTUATORNODEVELVET_H
