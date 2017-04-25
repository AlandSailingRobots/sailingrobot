#include "LowLevelControllerNodeASPire.h"

LowLevelControllerNodeASPire::LowLevelControllerNodeASPire(MessageBus& msgBus, CANService &canService, float maxRudderAngle = 30, 
                                                     float maxCourseAngleDiff = 60, float maxServoSailAngle = 10, float servoSailMinAngleDiff = 5) :
                                                     Node(NodeID::LowLevelControllerNodeASPire, msgBus), m_CanService(canService), m_MaxRudderAngle(maxRudderAngle),
                                                     m_MaxCourseAngleDiff(maxCourseAngleDiff), m_MaxServoSailAngle(maxServoSailAngle), m_ServoSailMinAngleDiff(servoSailMinAngleDiff)
{
    msgBus.registerNode(*this, MessageType::NavigationControl);
    msgBus.registerNode(*this, MessageType::WindState);
    msgBus.registerNode(*this, MessageType::StateMessage);
}


bool LowLevelControllerNodeASPire::init() { return true; }

void LowLevelControllerNodeASPire::processMessage(const Message* message){
    MessageType type = message->messageType();

    if(type == MessageType::StateMessage){
        processStateMessage(static_cast<const StateMessage*> (message));
    } else if(type == MessageType::WindState) {
        processWindStateMessage(static_cast<const WindStateMsg*> (message));
    } else if(type == MessageType::NavigationControl){
        processNavigationControlMessage(static_cast<const NavigationControlMsg*> (message));
    }

}

void LowLevelControllerNodeASPire::processStateMessage(const StateMessage* msg){
    float  m_VesselHeading = msg->heading();
    double m_VesselLatitude = msg->latitude();
    double m_VesselLongitude = msg->longitude();
    double m_VesselSpeed = msg->speed();
    double m_VesselCourse = msg->course();   
}

void LowLevelControllerNodeASPire::processWindStateMessage(const WindStateMsg* msg){
    double m_TrueWindSpeed = msg->trueWindSpeed();
	double m_TrueWindDir = msg->trueWindDirection();
	double m_ApparentWindSpeed = msg->apparentWindSpeed();
	double m_ApparentWindDir = msg->apparentWindDirection();
}

void LowLevelControllerNodeASPire::processNavigationControlMessage(const NavigationControlMsg* msg);