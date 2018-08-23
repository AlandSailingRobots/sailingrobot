
/****************************************************************************************
 *
 * File:
 * 		ActuatorNodeVelvet.cpp
 *
 * Purpose:
 *		Controls an actuator on the vessel.
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/

#include "../Hardwares/ActuatorNodeVelvet.h"
#include "../Messages/VelvetActuatorFeedbackMsg.h"
#include "../Hardwares/MaestroController/MaestroController.h"
#include "../SystemServices/Logger.h"
#include "../Messages/SailCommandMsg.h"
#include "../Messages/RudderCommandMsg.h"


ActuatorNodeVelvet::ActuatorNodeVelvet(MessageBus& msgBus, NodeID id, int channel, int speed, int acceleration)
        :Node(id, msgBus), m_Channel(channel), m_Speed(speed), m_Acceleration(acceleration)
{
    msgBus.registerNode(*this,MessageType::VelvetActuatorFeedback);
    msgBus.registerNode(*this,MessageType::ServerConfigsReceived);
}

ActuatorNodeVelvet::ActuatorNodeVelvet(MessageBus& msgBus, NodeID id, int channel, int speed, int acceleration, int minSailAngle, int maxSailAngle)
        :Node(id, msgBus), m_Channel(channel), m_Speed(speed), m_Acceleration(acceleration), m_minSailAngle(minSailAngle),
         m_maxSailAngle(maxSailAngle)
{
    //msgBus.registerNode(*this,MessageType::VelvetActuatorFeedback);
    msgBus.registerNode(*this,MessageType::SailCommand);
    msgBus.registerNode(*this,MessageType::ServerConfigsReceived);
}

ActuatorNodeVelvet::ActuatorNodeVelvet(MessageBus& msgBus, NodeID id, int channel, int speed, int acceleration, int maxRudderAngle)
        :Node(id, msgBus), m_Channel(channel), m_Speed(speed), m_Acceleration(acceleration), m_maxRudderAngle(maxRudderAngle)
{
    //msgBus.registerNode(*this,MessageType::VelvetActuatorFeedback);
    msgBus.registerNode(*this,MessageType::RudderCommand);
    msgBus.registerNode(*this,MessageType::ServerConfigsReceived);
}

bool ActuatorNodeVelvet::init()
{
    if( MaestroController::writeCommand(MaestroCommands::SetSpeed, m_Channel, m_Speed) &&
        MaestroController::writeCommand(MaestroCommands::SetAcceleration, m_Channel, m_Acceleration) )
    {
        return true;
    }
    else
    {
        Logger::error("%s Failed to write actuator speed and acceleration!", __PRETTY_FUNCTION__);
        return false;
    }

}

void ActuatorNodeVelvet::processMessage(const Message* message)
{
   /* if(message->messageType() == MessageType::VelvetActuatorFeedback)
    {
        VelvetActuatorFeedbackMsg* msg = (VelvetActuatorFeedbackMsg*)message;

        //Get relevant command
        int setPosition = 0;

        if (nodeID() == NodeID::RudderActuator)
        {
            setPosition = msg->rudderFeedback();
        }
        else if (nodeID() == NodeID::SailActuator)
        {
            setPosition = msg->sailFeedback();
        }
        else
        {
            Logger::warning("%s Actuator: %d Unknown/Unregistered actuator message NodeID", __PRETTY_FUNCTION__, (int)nodeID());
            return;
        }

        if(not MaestroController::writeCommand(MaestroCommands::SetPosition, m_Channel, setPosition))
        {
            Logger::error("%s Actuator: %d Failed to write position command", __PRETTY_FUNCTION__, (int)nodeID());
        }
    }*/

    // maybe init setPosition before and add a check on setPosition value before writing it
    // adding a looptime could be useful, have to check when testing
    if(message->messageType() == MessageType::SailCommand)
    {
        SailCommandMsg* msg = (SailCommandMsg*)message;
        int setPosition = 6000; // pwm value for sailwinch and rudder servo in middle course (sail in, rudder centered)
        if (nodeID() == NodeID::SailActuator)
        {
            setPosition = mapCommandToPWM(msg->maxSailAngle()); //have to check why it is named maxSailAngle
            // and modify it for smartwinch
        }

        MaestroController::writeCommand(MaestroCommands::SetPosition, m_Channel, setPosition);
       

    }

    if(message->messageType() == MessageType::RudderCommand)
    {
        RudderCommandMsg* msg = (RudderCommandMsg*)message;
        int setPosition = 6000; // pwm value for sailwinch and rudder servo in middle course (sail in, rudder centered)
        if (nodeID() == NodeID::RudderActuator)
        {
            setPosition = mapCommandToPWM(msg->rudderAngle()); //have to check why it is named maxSailAngle
            // and modify it for smartwinch
        }

        MaestroController::writeCommand(MaestroCommands::SetPosition, m_Channel, setPosition);


    }

}

float ActuatorNodeVelvet::mapCommandToPWM(float angleCommand) {
    int pwm_output = -1;
    int max_pwm = 0;
    int min_pwm = 0;
    int max_angle = 0;
    int min_angle = 0;
    if (nodeID() == NodeID::SailActuator)
    {
        min_pwm = MIN_PWM_SAIL;
        max_pwm = MAX_PWM_SAIL;
        min_angle = m_minSailAngle;
        max_angle = m_maxSailAngle;
    }
    else if (nodeID() == NodeID::RudderActuator)
    {
        min_pwm = MIN_PWM_RUDDER;
        max_pwm = MAX_PWM_RUDDER;
        max_angle = m_maxRudderAngle;
    }
    else
    {
        Logger::error("In ActuatorNodeVelvet: Wrong node catching an actuator command");
    }
    pwm_output = min_pwm + (max_pwm - min_pwm) * (angleCommand - min_angle)/(max_angle - min_angle);
    return pwm_output;
}