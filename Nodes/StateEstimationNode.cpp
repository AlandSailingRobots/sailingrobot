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

#define STATE_SLEEP_MS 400
#define STATE_INITIAL_SLEEP 2000

#define STEADY_CLOCK std::chrono::steady_clock
#define DURATION_CAST std::chrono::duration_cast
#define DURATION std::chrono::duration

StateEstimationNode::StateEstimationNode(MessageBus& msgBus, double mloopTime): ActiveNode(NodeID::StateEstimation, msgBus),
vesselHeading(0), vesselLat(0), vesselLon(0),vesselSpeed(0), vesselCourse(0), loopTime(mloopTime), declination(0)
{
  msgBus.registerNode(*this, MessageType::CompassData);
  msgBus.registerNode(*this, MessageType::GPSData);
  msgBus.registerNode(*this, MessageType::WaypointData);

}

StateEstimationNode::~StateEstimationNode()
{
  server.shutdown();
}

bool StateEstimationNode::init()
{
  return server.start( 9600 );
}

void StateEstimationNode::start()
{
  runThread(StateEstimationNodeThreadFunc);
}

void StateEstimationNode::processMessage(const Message* msg)
{
  MessageType type = msg->messageType();
  switch(type)
  {
    case MessageType::CompassData:
    processCompassMessage((CompassDataMsg*)msg);
    break;
    case MessageType::GPSData:
    processGPSMessage((GPSDataMsg*)msg);
    break;
    default:
    return;
  }
}

void StateEstimationNode::processCompassMessage(CompassDataMsg* msg)
{
  float currentVesselHeading = msg->heading();
  vesselHeading = Utility::addDeclinationToHeading(currentVesselHeading, declination);
}

void StateEstimationNode::processGPSMessage(GPSDataMsg* msg)
{
  vesselLat = msg->latitude();
  vesselLon = msg->longitude();
  vesselSpeed = msg->speed();
}

int StateEstimationNode::getCourse(){

/* Depending on the current speed (Speed over ground) use vesselHeading
 * (Compass heading compasated for declination)
 * or the GPS Course */
  if(vesselSpeed >= 0 && vesselSpeed <= SPEEDLIMIT){
    return (SPEEDLIMIT - vesselSpeed)/SPEEDLIMIT * vesselHeading + vesselSpeed/SPEEDLIMIT*vesselCourse;
  }else{
    return vesselCourse;
  }
}

void StateEstimationNode::processWaypointMessage( WaypointDataMsg* msg )
{
  declination = msg->nextDeclination();
}

void StateEstimationNode::StateEstimationNodeThreadFunc(void* nodePtr)
{
  StateEstimationNode* node = (StateEstimationNode*)nodePtr;

  // An initial sleep, its purpose is to ensure that most if not all the sensor data arrives
  // at the start before we send out the vessel state message.
  std::this_thread::sleep_for(std::chrono::milliseconds(STATE_INITIAL_SLEEP));

  char buffer[1024];

  auto startTime = STEADY_CLOCK::now().time_since_epoch();
  auto timeSinceStartTimePlusDelayEpoch = DURATION_CAST<DURATION<double>>(startTime).count();

  while(true)
  {
    // Listen to new connections
    node->server.acceptConnections();

    auto currentTime = STEADY_CLOCK::now().time_since_epoch();
    auto timeSinceCurrentTimeEpoch = DURATION_CAST<DURATION<double>>(currentTime).count();
    timeSinceStartTimePlusDelayEpoch += node->loopTime;
    int sleepTime = timeSinceStartTimePlusDelayEpoch - timeSinceCurrentTimeEpoch;

    // Controls how often we pump out messages
    if(sleepTime > 0){
      std::this_thread::sleep_for(std::chrono::milliseconds(sleepTime));
    }

    MessagePtr stateMessage = std::make_unique<StateMessage>(	node->vesselHeading, node->vesselLat,
      node->vesselLon, node->vesselSpeed, node->getCourse());
      node->m_MsgBus.sendMessage(std::move(stateMessage));

      // Compass heading, Speed, GPS Lat, GPS Lon
      int size = snprintf( buffer, 1024, "%d,%f,%f,%f\n",
      (int)node->vesselHeading,
      (float)node->vesselSpeed,
      node->vesselLat,
      node->vesselLon);
      if( size > 0 )
      {
        node->server.broadcast( (uint8_t*)buffer, size );
      }
    }
  }
