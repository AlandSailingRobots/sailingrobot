#pragma once

#include "Nodes/Node.h"
#include "Messages/WindStateMsg.h"
#include "Messages/StateMessage.h"
#include "waypointrouting/SailCommand.h"
#include "Messages/NavigationControlMsg.h"
#include "Messages/MessageTypes.h"
#include "MessageBus/MessageBus.h"
#include "HardwareServices/CAN_Services/CANService.h"
#include "SystemServices/WingsailControl.h"
#include "SystemServices/CourseRegulator.h"

#define DATA_OUT_OF_RANGE -2000

class LowLevelControllerJanet: public Node {
public:
  LowLevelControllerJanet(MessageBus& msgBus, float maxRudderAngle = 30,
   float maxCourseAngleDiff = 60);

    virtual ~LowLevelControllerJanet();
    bool init();
    void processMessage(const Message* message);

  private:

    void processStateMessage(const StateMessage* msg);
    void processNavigationControlMessage(const NavigationControlMsg* msg);
    void processWindStateMessage(const WindStateMsg* msg);
    void sendActuatorPosition();

    double  m_maxCommandAngle, m_maxSailAngle, m_minSailAngle;

    double m_ApparentWindDir = DATA_OUT_OF_RANGE;

    NavigationControlMsg::NavigationState m_NavigationState;


    const float m_MaxRudderAngle;

    CourseRegulator m_CourseRegulator;
    SailCommand m_SailCommand;
  };
