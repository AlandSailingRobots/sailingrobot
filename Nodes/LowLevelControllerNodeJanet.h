#pragma once

#include "Nodes/Node.h"
#include "Messages/WindStateMsg.h"
#include "Messages/StateMessage.h"
#include "Messages/ActuatorPositionMsg.h"
#include "Messages/NavigationControlMsg.h"
#include "Messages/MessageTypes.h"
#include "MessageBus/MessageBus.h"
#include "SystemServices/CourseRegulator.h"
#include "SystemServices/SoftSailControl.h"


class LowLevelControllerNodeJanet: public Node {
public:
  LowLevelControllerNodeJanet(MessageBus& msgBus, float maxRudderAngle = 30,
   float maxCourseAngleDiff = 60);

    virtual ~LowLevelControllerNodeJanet();
    bool init();
    void processMessage(const Message* message);

  private:

    void processStateMessage(const StateMessage* msg);
    void processNavigationControlMessage(const NavigationControlMsg* msg);
    void processWindStateMessage(const WindStateMsg* msg);
    void sendActuatorPosition();

    double  m_maxCommandAngle, m_maxSailAngle, m_minSailAngle;

    double m_ApparentWindDir = DATA_OUT_OF_RANGE;
    double m_VesselCourse = DATA_OUT_OF_RANGE;
    int m_CourseToSteer = DATA_OUT_OF_RANGE;


    NavigationState m_NavigationState;

    const float m_MaxRudderAngle;

    CourseRegulator m_CourseRegulator;
    SoftSailControl m_SailCommand;

    const int DATA_OUT_OF_RANGE = -2000;

  };
