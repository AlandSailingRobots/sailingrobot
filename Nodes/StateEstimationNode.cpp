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

  #define STATE_SLEEP_MS 400
  #define STATE_INITIAL_SLEEP 2000

  StateEstimationNode::StateEstimationNode(MessageBus& msgBus, double mloopTime): ActiveNode(NodeID::StateEstimation, msgBus),
  vesselHeading(0), vesselLat(0), vesselLon(0),vesselSpeed(0), vesselCourse(0), loopTime(mloopTime), declination(0), speedLimit(0)
  {
    msgBus.registerNode(*this, MessageType::CompassData);
    msgBus.registerNode(*this, MessageType::GPSData);
    msgBus.registerNode(*this, MessageType::WaypointData);

  }

  StateEstimationNode::~StateEstimationNode()
  {

  }

  bool StateEstimationNode::init()
  {
    return true;
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
      case MessageType::WaypointData:
      processWaypointMessage((WaypointDataMsg*)msg);
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
    vesselCourse = msg->heading();
  }

  int StateEstimationNode::getCourse(){

  /* Depending on the current speed (Speed over ground) use vesselHeading
   * (Compass heading compensated with declination)
   * or the GPS Course */
    if(vesselSpeed >= 0 && vesselSpeed <= speedLimit){
      int leftOperand = ( (speedLimit-vesselSpeed)/speedLimit )* vesselHeading;
      int rightOperand = (vesselSpeed/speedLimit)*vesselCourse;
      return leftOperand + rightOperand;
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


    Timer timer;
    timer.start();

   
    while(true)
    {
      // Listen to new connections
      node->server.acceptConnections();

      timer.sleepUntil(node->loopTime);


      MessagePtr stateMessage = std::make_unique<StateMessage>(	node->vesselHeading, node->vesselLat,
        node->vesselLon, node->vesselSpeed, node->getCourse());
        node->m_MsgBus.sendMessage(std::move(stateMessage));
      }
       timer.reset();

    }

