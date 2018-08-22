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

#define MIN_PWM_SAIL 4500
#define MAX_PWM_SAIL 5500
#define MIN_PWM_RUDDER 4500
#define MAX_PWM_RUDDER 5500

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

    ///----------------------------------------------------------------------------------
    /// Map angle commands into pwm value
    ///
    ///----------------------------------------------------------------------------------
    int mapCommandToPWM(int angleCommand);

private:
    int m_Channel;
    int m_Speed;
    int m_Acceleration;
};



#endif //SAILINGROBOT_ACTUATORNODEVELVET_H
