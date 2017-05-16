#include "LowLevelControllerJanet.h"
#include "HardwareServices/CAN_Services/N2kMsg.h"
#include "Math/Utility.h"
#include "Messages/ActuatorPositionMsg.h"

#include <math.h>
#include <algorithm>
#include <cmath>

#define DATA_OUT_OF_RANGE -2000


LowLevelControllerJanet::LowLevelControllerJanet(MessageBus& msgBus,
  float maxRudderAngle,  float maxCourseAngleDiff) :
  Node(NodeID::LowLevelControllerNodeASPire, msgBus),m_MaxRudderAngle(maxRudderAngle),
  m_CourseRegulator(maxRudderAngle, maxCourseAngleDiff)
  {
    msgBus.registerNode(*this, MessageType::NavigationControl);
    msgBus.registerNode(*this, MessageType::StateMessage);
    msgBus.registerNode(*this, MessageType::WindState);
  }

  LowLevelControllerJanet::~LowLevelControllerJanet() {}

  bool LowLevelControllerJanet::init() { return true; }

  void LowLevelControllerJanet::processMessage(const Message* message){
    MessageType type = message->messageType();

    if(type == MessageType::StateMessage){
      processStateMessage(static_cast<const StateMessage*> (message));
    } else if(type == MessageType::NavigationControl){
      processNavigationControlMessage(static_cast<const NavigationControlMsg*> (message));
    }else if(type == MessageType::WindState){
      processNavigationControlMessage(static_cast<const NavigationControlMsg*> (message));
    }

    // make sure that the three messages are received atleast once before sending out a actuator position
    if(m_VesselCourse != DATA_OUT_OF_RANGE && m_CourseToSteer != DATA_OUT_OF_RANGE && m_ApparentWindDir != DATA_OUT_OF_RANGE) {
        sendActuatorPosition();
    }
  }

  void LowLevelControllerJanet::processStateMessage(const StateMessage* msg){
    m_VesselCourse = msg->course();
    m_CourseRegulator.setVesselCourse(m_VesselCourse);
  }

  void LowLevelControllerJanet::processNavigationControlMessage(const NavigationControlMsg* msg){
    m_CourseToSteer = msg->courseToSteer();
    m_CourseRegulator.setCourseToSteer(m_CourseToSteer);
  }

  void LowLevelControllerJanet::processWindStateMessage(const WindStateMsg* msg){
  	m_ApparentWindDir   = msg->apparentWindDirection();
  }

  void LowLevelControllerJanet::sendActuatorPosition(){

    double normalizedRudderAngle = m_CourseRegulator.calculateRudderAngle() / m_MaxRudderAngle;
    double sailCommand = m_SailCommand.getSailCommand(m_ApparentWindDir);

    MessagePtr actuatorMsg = std::make_unique<ActuatorPositionMsg>(normalizedRudderAngle, sailCommand);
    m_MsgBus.sendMessage(std::move(actuatorMsg));
  }
