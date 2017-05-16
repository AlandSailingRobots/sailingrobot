#include "LowLevelControllerJanet.h"
#include "HardwareServices/CAN_Services/N2kMsg.h"
#include "Math/Utility.h"

#include <math.h>
#include <algorithm>
#include <cmath>

LowLevelControllerJanet::LowLevelControllerJanet(MessageBus& msgBus, CANService& canService, float maxRudderAngle,
  float maxCourseAngleDiff, float maxServoSailAngle, float servoSailMinAngleDiff) :
  Node(NodeID::LowLevelControllerNodeASPire, msgBus),
  m_MaxRudderAngle(maxRudderAngle), m_MaxServoSailAngle(maxServoSailAngle),
  m_CanService(&canService),
  m_WingsailControl(servoSailMinAngleDiff, maxServoSailAngle), m_CourseRegulator(maxRudderAngle, maxCourseAngleDiff)

  {
    msgBus.registerNode(*this, MessageType::NavigationControl);
    msgBus.registerNode(*this, MessageType::StateMessage);
    msgBus.registerNode(*this, MessageType::WindState);

    m_maxCommandAngle = M_PI / 6;
    m_maxSailAngle = M_PI / 4.2f;
    m_minSailAngle = M_PI / 32.0f;
    m_tackAngle = 0.872665; //50°
  }

  LowLevelControllerJanet::~LowLevelControllerJanet() {}

  bool LowLevelControllerJanet::init() { return true; }

  void LowLevelControllerJanet::processMessage(const Message* message){
    MessageType type = message->messageType();

    if(type == MessageType::StateMessage){
      processStateMessage(static_cast<const StateMessage*> (message));
    } else if(type == MessageType::NavigationControl){
      processNavigationControlMessage(static_cast<const NavigationControlMsg*> (message));
    }

    // Check that we have received atleast one of every message so far
    if(m_VesselHeading != DATA_OUT_OF_RANGE && m_CourseToSteer != DATA_OUT_OF_RANGE) {
      constructAndSendFrame();
    }
  }

  void LowLevelControllerJanet::processStateMessage(const StateMessage* msg){
    m_VesselHeading   = msg->heading();
    m_VesselLatitude  = msg->latitude();
    m_VesselLongitude = msg->longitude();
    m_VesselSpeed     = msg->speed();
    m_VesselCourse    = msg->course();

    m_WingsailControl.setVesselHeading(msg->heading());
    m_CourseRegulator.setVesselCourse(msg->course());
  }

  void LowLevelControllerJanet::processNavigationControlMessage(const NavigationControlMsg* msg){
    m_NavigationState   = msg->navigationState();
    m_CourseToSteer     = msg->courseToSteer();
    m_TargetSpeed       = msg->targetSpeed();
    m_WindvaneSelfSteeringOn = msg->windvaneSelfSteeringOn();

    m_CourseRegulator.setCourseToSteer(msg->courseToSteer());
  }

  void LowLevelControllerJanet::processWindStateMessage(const WindStateMsg* msg){
    m_TrueWindSpeed     = msg->trueWindSpeed();
  	m_TrueWindDir       = msg->trueWindDirection();
  	m_ApparentWindSpeed = msg->apparentWindSpeed();
  	m_ApparentWindDir   = msg->apparentWindDirection();

    m_WingsailControl.setTrueWindDirection(msg->trueWindDirection());
  }

  void LowLevelControllerJanet::constructAndSendFrame() {
    CanMsg Cmsg;
    Cmsg.id = 700;
    Cmsg.header.ide = 0;
    Cmsg.header.length = 8;

    double rudderAngle = m_CourseRegulator.calculateRudderAngle();
    rudderAngle += m_MaxRudderAngle;
    double ratio = 65535 / m_MaxRudderAngle * 2;
    uint16_t angle_16 = rudderAngle * ratio;

    (Cmsg.data[0] = angle_16 & 0xff);
    (Cmsg.data[1] = angle_16 >> 8);

    double servoAngle = m_WingsailControl.calculateServoAngle();
    servoAngle += m_MaxServoSailAngle;
    ratio = 65535 / m_MaxServoSailAngle * 2;
    angle_16 = servoAngle * ratio;

    (Cmsg.data[2] = angle_16 & 0xff);
    (Cmsg.data[3] = angle_16 >> 8);
    (Cmsg.data[4] = 0);
    (Cmsg.data[5] = 0);
    (Cmsg.data[6] = m_WindvaneSelfSteeringOn);
    (Cmsg.data[7] = 0);

    m_CanService->sendCANMessage(Cmsg);
  }

  void LowLevelControllerJanet::sendOldActuatorPositionMessage(){
    double rudderCommand, sailCommand;

    double currentHeading_radian = Utility::degreeToRadian(m_VesselHeading);

    //SET RUDDER-------
    if(cos(currentHeading_radian - m_CourseToSteer) < 0) //if boat heading is too far away from desired heading
    {
      rudderCommand = -Utility::sgn(m_VesselSpeed) * m_maxCommandAngle * Utility::sgn(sin(currentHeading_radian - m_CourseToSteer));
    }
    else
    {
      rudderCommand = -Utility::sgn(m_VesselSpeed) * m_maxCommandAngle * sin(currentHeading_radian - m_CourseToSteer);
    }

    //SET SAIL---------
    sailCommand = fabs(((m_minSailAngle - m_maxSailAngle) / M_PI) * fabs(m_ApparentWindDir) + m_maxSailAngle);/*!!! on some pc abs only ouptut an int (ubuntu 14.04 gcc 4.9.3)*/

    if (cos(m_ApparentWindDir + M_PI) + cos(m_maxSailAngle) <0 )
    {
      sailCommand = m_minSailAngle;
    }

  }
