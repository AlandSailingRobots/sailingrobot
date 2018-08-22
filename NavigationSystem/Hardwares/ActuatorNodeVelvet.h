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

#define MIN_PWM_SAIL 4000
#define MAX_PWM_SAIL 8000
#define MIN_PWM_RUDDER 4000
#define MAX_PWM_RUDDER 8000

//CHECK ANGLE

class ActuatorNodeVelvet : public Node {
public:
    ActuatorNodeVelvet(MessageBus& msgBus, NodeID id, int channel, int speed, int acceleration);
    ActuatorNodeVelvet(MessageBus& msgBus, NodeID id, int channel, int speed, int acceleration, int minSailAngle, int maxSailAngle);
    ActuatorNodeVelvet(MessageBus& msgBus, NodeID id, int channel, int speed, int acceleration, int maxRudderAngle);

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
    float mapCommandToPWM(float angleCommand);

private:
    int m_Channel;
    int m_Speed;
    int m_Acceleration;

    int m_minSailAngle;
    int m_maxSailAngle;
    int m_maxRudderAngle;

    int m_minSailAngle;
    int m_maxSailAngle;
    int m_maxRudderAngle;

};



#endif //SAILINGROBOT_ACTUATORNODEVELVET_H
