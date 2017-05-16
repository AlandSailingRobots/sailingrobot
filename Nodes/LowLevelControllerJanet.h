#pragma once

#include "Nodes/Node.h"
#include "Messages/WindStateMsg.h"
#include "Messages/StateMessage.h"
#include "Messages/NavigationControlMsg.h"
#include "Messages/MessageTypes.h"
#include "MessageBus/MessageBus.h"
#include "HardwareServices/CAN_Services/CANService.h"
#include "SystemServices/WingsailControl.h"
#include "SystemServices/CourseRegulator.h"

#define DATA_OUT_OF_RANGE -2000

class LowLevelControllerJanet: public Node {
public:
  LowLevelControllerJanet(MessageBus& msgBus, CANService &canService, float maxRudderAngle = 30,
    float maxCourseAngleDiff = 60, float maxServoSailAngle = 10, float servoSailMinAngleDiff = 5);

    virtual ~LowLevelControllerJanet();
    bool init();
    void processMessage(const Message* message);

  private:

    void processStateMessage(const StateMessage* msg);
    void processNavigationControlMessage(const NavigationControlMsg* msg);
    void processWindStateMessage(const WindStateMsg* msg);
    void constructAndSendFrame();
    void sendOldActuatorPositionMessage();

    double  m_maxCommandAngle, m_maxSailAngle, m_minSailAngle;
  	double  m_tackAngle;

    float  m_VesselHeading = DATA_OUT_OF_RANGE;
    double m_VesselLatitude = DATA_OUT_OF_RANGE;
    double m_VesselLongitude = DATA_OUT_OF_RANGE;
    double m_VesselSpeed = DATA_OUT_OF_RANGE;
    double m_VesselCourse = DATA_OUT_OF_RANGE;

    double m_TrueWindSpeed = DATA_OUT_OF_RANGE;
    double m_TrueWindDir = DATA_OUT_OF_RANGE;
    double m_ApparentWindSpeed = DATA_OUT_OF_RANGE;
    double m_ApparentWindDir = DATA_OUT_OF_RANGE;

    NavigationControlMsg::NavigationState m_NavigationState;
    int m_CourseToSteer = DATA_OUT_OF_RANGE;
    float m_TargetSpeed = DATA_OUT_OF_RANGE;
    bool m_WindvaneSelfSteeringOn = false;

    const float m_MaxRudderAngle;
    const float m_MaxServoSailAngle;

    CANService* m_CanService;
    WingsailControl m_WingsailControl;
    CourseRegulator m_CourseRegulator;
  };
