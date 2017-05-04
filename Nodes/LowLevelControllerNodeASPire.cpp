#include "LowLevelControllerNodeASPire.h"
#include "HardwareServices/CAN_Services/N2kMsg.h"


LowLevelControllerNodeASPire::LowLevelControllerNodeASPire(MessageBus& msgBus, CANService& canService, float maxRudderAngle = 30, 
                                                     float maxCourseAngleDiff = 60, float maxServoSailAngle = 10, float servoSailMinAngleDiff = 5) :
                                                     Node(NodeID::LowLevelControllerNodeASPire, msgBus), m_CanService(&canService),
                                                     m_WingsailControl(servoSailMinAngleDiff, maxServoSailAngle), m_CourseRegulator(maxRudderAngle, maxCourseAngleDiff)
{
    msgBus.registerNode(*this, MessageType::NavigationControl);
    msgBus.registerNode(*this, MessageType::WindState);
    msgBus.registerNode(*this, MessageType::StateMessage);
}

LowLevelControllerNodeASPire::~LowLevelControllerNodeASPire() {}

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
    m_VesselHeading   = msg->heading();
    m_VesselLatitude  = msg->latitude();
    m_VesselLongitude = msg->longitude();
    m_VesselSpeed     = msg->speed();
    m_VesselCourse    = msg->course();   

    m_WingsailControl.setVesselHeading(msg->heading());
    m_CourseRegulator.setVesselCourse(msg->course());
}

void LowLevelControllerNodeASPire::processWindStateMessage(const WindStateMsg* msg){
    m_TrueWindSpeed     = msg->trueWindSpeed();
	m_TrueWindDir       = msg->trueWindDirection();
	m_ApparentWindSpeed = msg->apparentWindSpeed();
	m_ApparentWindDir   = msg->apparentWindDirection();

    m_WingsailControl.setTrueWindDirection(msg->trueWindDirection());
}

void LowLevelControllerNodeASPire::processNavigationControlMessage(const NavigationControlMsg* msg){
    m_NavigationState   = msg->navigationState();
    m_CourseToSteer     = msg->courseToSteer();
    m_TargetSpeed       = msg->targetSpeed();
    m_WindvaneSelfSteeringOn = msg->windvaneSelfSteeringOn();

    m_CourseRegulator.setCourseToSteer(msg->courseToSteer());
}
