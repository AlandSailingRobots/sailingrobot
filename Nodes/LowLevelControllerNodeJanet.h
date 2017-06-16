#pragma once

#include "MessageBus/Node.h"
#include "Messages/WindStateMsg.h"
#include "Messages/StateMessage.h"
#include "Messages/ActuatorPositionMsg.h"
#include "Messages/NavigationControlMsg.h"
#include "MessageBus/MessageTypes.h"
#include "MessageBus/MessageBus.h"
#include "SystemServices/CourseRegulator.h"
#include "SystemServices/SoftsailControl.h"
#include "DataBase/DBHandler.h"



class LowLevelControllerNodeJanet: public Node {
public:
  LowLevelControllerNodeJanet(MessageBus& msgBus, float maxRudderAngle,
   float maxCourseAngleDiff, DBHandler& db);

    virtual ~LowLevelControllerNodeJanet();
    bool init();
    void processMessage(const Message* message);

  private:

    void processStateMessage(const StateMessage* msg);
    void processNavigationControlMessage(const NavigationControlMsg* msg);
    void processWindStateMessage(const WindStateMsg* msg);
    void sendActuatorPosition();
    void setupSailCommand();


    double  m_maxCommandAngle, m_maxSailAngle, m_minSailAngle;

    double m_ApparentWindDir = DATA_OUT_OF_RANGE;
    double m_VesselCourse = DATA_OUT_OF_RANGE;
    int m_CourseToSteer = DATA_OUT_OF_RANGE;


    NavigationState m_NavigationState;

    const float m_MaxRudderAngle;

    CourseRegulator m_CourseRegulator;
    SoftsailControl m_SailCommand;

    const int DATA_OUT_OF_RANGE = -2000;
    DBHandler &m_db;
  };
