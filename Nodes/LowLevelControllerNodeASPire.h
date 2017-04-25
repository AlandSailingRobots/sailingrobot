#pragma once

#include "Nodes/Node.h"
#include "Messages/WindStateMsg.h"
#include "Messages/StateMessage.h"
#include "MessageBus/MessageBus.h"
#include "HardwareServices/CAN_Services/CANService.h"

class LowLevelControllerNodeASPire : public Node {
public:
    LowLevelControllerNodeASPire(MessageBus& msgBus, CANService &canService, float maxRudderAngle, 
                                    float maxCourseAngleDiff, float maxServoSailAngle, float servoSailMinAngleDiff);

    virtual ~LowLevelControllerNodeASPire();
    bool init();
    void processMessage(const Message* message);

private:

    void processStateMessage(const StateMessage* msg);
    void processWindStateMessage(const WindStateMsg* msg);
    void processNavigationControlMessage(const NavigationControlMsg* msg);

    float m_MaxRudderAngle;
    float m_MaxCourseAngleDiff;
    float m_MaxServoSailAngle;
    float m_ServoSailMinAngleDiff;

    float  m_VesselHeading;
    double m_VesselLatitude;
    double m_VesselLongitude;
    double m_VesselSpeed;
    double m_VesselCourse;

    double m_TrueWindSpeed;
	double m_TrueWindDir;
	double m_ApparentWindSpeed;
	double m_ApparentWindDir;
}