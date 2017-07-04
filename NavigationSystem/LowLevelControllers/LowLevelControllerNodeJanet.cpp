#include "LowLevelControllerNodeJanet.h"
#include "Math/Utility.h"

#include <math.h>
#include <algorithm>
#include <cmath>



LowLevelControllerNodeJanet::LowLevelControllerNodeJanet(MessageBus& msgBus,
  float maxRudderAngle,  float maxCourseAngleDiff, DBHandler& db):
  Node(NodeID::LowLevelControllerNodeJanet, msgBus),m_MaxRudderAngle(maxRudderAngle),
  m_CourseRegulator(maxRudderAngle, maxCourseAngleDiff), m_db(db) 
  {
    msgBus.registerNode(*this, MessageType::NavigationControl);
    msgBus.registerNode(*this, MessageType::StateMessage);
    msgBus.registerNode(*this, MessageType::WindState);
  }

  LowLevelControllerNodeJanet::~LowLevelControllerNodeJanet() {}

  bool LowLevelControllerNodeJanet::init() {
    setupSailCommand();
    return true;
  }

  void LowLevelControllerNodeJanet::processMessage(const Message* message){
    MessageType type = message->messageType();

    if(type == MessageType::StateMessage){
      processStateMessage(static_cast<const StateMessage*> (message));
    } else if(type == MessageType::NavigationControl){
      processNavigationControlMessage(static_cast<const NavigationControlMsg*> (message));
    }else if(type == MessageType::WindState){
      processWindStateMessage(static_cast<const WindStateMsg*> (message));
    }
    // make sure that the three messages are received atleast once before sending out a actuator position
    if(m_VesselCourse != DATA_OUT_OF_RANGE && m_CourseToSteer != DATA_OUT_OF_RANGE && m_ApparentWindDir != DATA_OUT_OF_RANGE) {
      sendActuatorPosition();
    }
  }

  void LowLevelControllerNodeJanet::processStateMessage(const StateMessage* msg){
    m_VesselCourse = msg->course();
    m_CourseRegulator.setVesselCourse(m_VesselCourse);
  }

  void LowLevelControllerNodeJanet::processNavigationControlMessage(const NavigationControlMsg* msg){
    m_CourseToSteer = msg->courseToSteer();
    m_CourseRegulator.setCourseToSteer(m_CourseToSteer);
  }

  void LowLevelControllerNodeJanet::processWindStateMessage(const WindStateMsg* msg){
    m_ApparentWindDir = msg->apparentWindDirection();
  }

  void LowLevelControllerNodeJanet::sendActuatorPosition(){

    double normalizedRudderAngle = m_CourseRegulator.calculateRudderAngle() / m_MaxRudderAngle;
    double sailCommand = m_SailCommand.getSailCommand(m_ApparentWindDir);

    MessagePtr actuatorMsg = std::make_unique<ActuatorPositionMsg>(normalizedRudderAngle, sailCommand);
    m_MsgBus.sendMessage(std::move(actuatorMsg));
  }

  void LowLevelControllerNodeJanet::setupSailCommand() {
    m_SailCommand.setCommandValues( m_db.retrieveCellAsInt("sail_command_config", "1", "close_reach_command"),
	        m_db.retrieveCellAsInt("sail_command_config", "1", "run_command"));
  }
