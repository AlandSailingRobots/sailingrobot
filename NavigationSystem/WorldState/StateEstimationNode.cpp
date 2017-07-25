/****************************************************************************************
*
* File:
* 		StateEstimationNode.cpp
*
* Purpose:
* Maintains the "current" state listening to GPS and compass messages, sending out a
* StateMessage
*
* Developer Notes:
*
*
***************************************************************************************/

#include "StateEstimationNode.h"

// For std::this_thread
#include <chrono>
#include <thread>

#include "Messages/StateMessage.h"
#include "Math/CourseMath.h"
#include "SystemServices/Logger.h"
#include "Math/Utility.h"
#include "SystemServices/Timer.h"

StateEstimationNode::StateEstimationNode(MessageBus& msgBus, DBHandler& db ,double loopTime, double speedLimit): ActiveNode(NodeID::StateEstimation, msgBus),
m_VesselHeading(0), m_VesselLat(0), m_VesselLon(0), m_VesselSpeed(0), m_VesselCourse(0), m_LoopTime(loopTime), m_Declination(0),
m_SpeedLimit(speedLimit), m_GpsOnline(false), m_dbHandler(db)
{
  msgBus.registerNode(*this, MessageType::CompassData);
  msgBus.registerNode(*this, MessageType::GPSData);
  msgBus.registerNode(*this, MessageType::WaypointData);
  msgBus.registerNode(*this, MessageType::ServerConfigsReceived);
}

StateEstimationNode::~StateEstimationNode() {}

bool StateEstimationNode::init()
{
  return true;
}

void StateEstimationNode::start()
{
  runThread(StateEstimationNodeThreadFunc);
}

void StateEstimationNode::updateConfigsFromDB()
{
    m_LoopTime = m_dbHandler.retrieveCellAsDouble("config_vesselState","1","loop_time");
    m_SpeedLimit = m_dbHandler.retrieveCellAsDouble("config_vesselState","1","speedLimit");
}

void StateEstimationNode::processMessage(const Message* msg)
{
  MessageType type = msg->messageType();
  switch(type)
  {
    case MessageType::CompassData:
    processCompassMessage(static_cast<const CompassDataMsg*>(msg));
    break;
    case MessageType::GPSData:
    processGPSMessage(static_cast<const GPSDataMsg*> (msg));
    break;
    case MessageType::WaypointData:
    processWaypointMessage(static_cast<const WaypointDataMsg*> (msg));
    break;
    case MessageType::ServerConfigsReceived:
    updateConfigsFromDB();
    break;
    default:
    return;
  }
}

void StateEstimationNode::processCompassMessage(const CompassDataMsg* msg)
{
  std::lock_guard<std::mutex> lock_guard(m_lock);
  float currentVesselHeading = msg->heading();
  m_VesselHeading = Utility::addDeclinationToHeading(currentVesselHeading, m_Declination);
}

void StateEstimationNode::processGPSMessage(const GPSDataMsg* msg)
{
  std::lock_guard<std::mutex> lock_guard(m_lock);
  m_VesselLat = msg->latitude();
  m_VesselLon = msg->longitude();
  m_VesselSpeed = msg->speed();
  m_VesselCourse = msg->heading();
  m_GpsOnline = msg->gpsOnline();
}

int StateEstimationNode::getCourse(){

  /*
  * Depending on the current speed (Speed over ground) use vesselHeading
  * (Compass heading compensated with declination)
  * or the GPS Course
  */
  if(m_VesselSpeed >= 0 && m_VesselSpeed <= m_SpeedLimit){
    int leftOperand = ( (m_SpeedLimit-m_VesselSpeed)/m_SpeedLimit )* m_VesselHeading;
    int rightOperand = (m_VesselSpeed/m_SpeedLimit)*m_VesselCourse;
    return leftOperand + rightOperand;
  }
  return m_VesselCourse;
}

void StateEstimationNode::processWaypointMessage( const WaypointDataMsg* msg )
{
  std::lock_guard<std::mutex> lock_guard(m_lock);
  m_Declination = msg->nextDeclination();
}

void StateEstimationNode::StateEstimationNodeThreadFunc(ActiveNode* nodePtr)
{
  StateEstimationNode* node = dynamic_cast<StateEstimationNode*> (nodePtr);

  // An initial sleep, its purpose is to ensure that most if not all the sensor data arrives
  // at the start before we send out the state message.
  std::this_thread::sleep_for(std::chrono::milliseconds(node->STATE_INITIAL_SLEEP));

  Timer timer;
  timer.start();

  while(true)
  {
    if(node->m_GpsOnline){
      std::lock_guard<std::mutex> lock_guard(node->m_lock);
      MessagePtr stateMessage = std::make_unique<StateMessage>(node->m_VesselHeading, node->m_VesselLat,
      node->m_VesselLon, node->m_VesselSpeed, node->getCourse());
      node->m_MsgBus.sendMessage(std::move(stateMessage));
    }
    timer.sleepUntil(node->m_LoopTime);
    timer.reset();
  }

}
