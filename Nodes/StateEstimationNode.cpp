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

      #define STATE_INITIAL_SLEEP 2000

      StateEstimationNode::StateEstimationNode(MessageBus& msgBus, double loopTime, double speedLimit): ActiveNode(NodeID::StateEstimation, msgBus),
      mVesselHeading(0), mVesselLat(0), mVesselLon(0), mVesselSpeed(0), mVesselCourse(0), mLoopTime(loopTime), mDeclination(0), mSpeedLimit(speedLimit)
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
          processCompassMessage(static_cast<const CompassDataMsg*>(msg));
          break;
          case MessageType::GPSData:
          processGPSMessage(static_cast<const GPSDataMsg*> (msg));
          break;
          case MessageType::WaypointData:
          processWaypointMessage(static_cast<const WaypointDataMsg*> (msg));
          break;
          default:
          return;
        }
      }

      void StateEstimationNode::processCompassMessage(const CompassDataMsg* msg)
      {
        m_lock.lock();
        float currentVesselHeading = msg->heading();
        mVesselHeading = Utility::addDeclinationToHeading(currentVesselHeading, mDeclination);
        m_lock.unlock();

      }

      void StateEstimationNode::processGPSMessage(const GPSDataMsg* msg)
      {
        m_lock.lock();
        mVesselLat = msg->latitude();
        mVesselLon = msg->longitude();
        mVesselSpeed = msg->speed();
        mVesselCourse = msg->heading();
        m_lock.unlock();
      }

      int StateEstimationNode::getCourse(){

      /* Depending on the current speed (Speed over ground) use vesselHeading
       * (Compass heading compensated with declination)
       * or the GPS Course 
       */
        
        std::lock_guard<std::mutex> lock_guard(m_lock);
        if(mVesselSpeed >= 0 && mVesselSpeed <= mSpeedLimit){
          int leftOperand = ( (mSpeedLimit-mVesselSpeed)/mSpeedLimit )* mVesselHeading;
          int rightOperand = (mVesselSpeed/mSpeedLimit)*mVesselCourse;
          return leftOperand + rightOperand;
        }
          return mVesselCourse;
      }

      void StateEstimationNode::processWaypointMessage( const WaypointDataMsg* msg )
      {                  
        m_lock.lock();
        mDeclination = msg->nextDeclination();
        m_lock.unlock();

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

        timer.sleepUntil(node->mLoopTime);
        MessagePtr stateMessage = std::make_unique<StateMessage>( node->mVesselHeading, node->mVesselLat,
          node->mVesselLon, node->mVesselSpeed, node->getCourse());
          node->m_MsgBus.sendMessage(std::move(stateMessage));
          timer.reset();
      }

      }

