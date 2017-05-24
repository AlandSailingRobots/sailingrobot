#include "DBLoggerNode.h"

#include "Messages/WindStateMsg.h"
#include "Messages/ActuatorPositionMsg.h"
#include "Messages/ActuatorControlASPireMessage.h"
#include "Messages/CourseDataMsg.h"
#include "Messages/NavigationControlMsg.h"
#include "Messages/WaypointDataMsg.h"
#include "SystemServices/Timer.h"
#include "SystemServices/SysClock.h"


DBLoggerNode::DBLoggerNode(MessageBus& msgBus, DBHandler& db, int TimeBetweenMsgs)
: ActiveNode(NodeID::DBLoggerNode, msgBus), m_db(db), m_dbLogger(5, db), m_TimeBetweenMsgs(TimeBetweenMsgs)
{
}

void DBLoggerNode::processMessage(const Message* msg) {

    std::lock_guard<std::mutex> lock(m_lock);

    MessageType type = msg->messageType();

    switch(type) 
    {
        case MessageType::WindState: 
        {
            const WindStateMsg* windStateMsg = static_cast<const WindStateMsg*>(msg);
            m_trueWindDir = windStateMsg->trueWindDirection();
        }
        break;

        case MessageType::ActuatorPosition:
        {
            const ActuatorPositionMsg* actuatorPositionMsg = static_cast<const ActuatorPositionMsg*>(msg);
            m_rudderCommand = actuatorPositionMsg->rudderPosition();
            m_sailCommand = actuatorPositionMsg->sailPosition();
        }
        break;

        case MessageType::ActuatorControlASPire:
        {
            const ActuatorControlASPireMessage* aspireMsg = static_cast<const ActuatorControlASPireMessage*>(msg);
            m_rudderCommand = aspireMsg->rudderAngle();
            m_sailCommand = aspireMsg->wingsailServoAngle();
        }
        break;

        case MessageType::CourseData:
        {
            const CourseDataMsg* courseDataMsg = static_cast<const CourseDataMsg*>(msg);
            m_distanceToNextWaypoint = courseDataMsg->distanceToWP();
            m_bearingToNextWaypoint = courseDataMsg->courseToWP();
        }
        break;

        case MessageType::NavigationControl:
        {
            const NavigationControlMsg* navigationControlMsg = static_cast<const NavigationControlMsg*>(msg);
            m_desiredHeading = navigationControlMsg->courseToSteer();
            m_tack = navigationControlMsg->tack();
            m_starboard = navigationControlMsg->starboard();
        }

        case MessageType::WaypointData:
        {
            WaypointDataMsg* waypMsg = (WaypointDataMsg*)msg;
            m_nextWaypointId = waypMsg->nextId();
        }

        default:
        return;
    }
}

void DBLoggerNode::start() {
    runThread(DBLoggerNodeThreadFunc);
}

void DBLoggerNode::DBLoggerNodeThreadFunc(ActiveNode* nodePtr) {

    DBLoggerNode* node = dynamic_cast<DBLoggerNode*> (nodePtr);
    Timer timer;
    timer.start();

    while(true) {

        timer.sleepUntil(node->m_TimeBetweenMsgs*1.0f / 1000);
        timer.reset();
        node->m_lock.lock();

        // TODO - insert into database

        // rudderCommand_norm & sailCommand_norm - ActuatorPositionMsg
        // rudderCommand_norm2, sailCommand_norm2 - ActuatorControlASPireMessage
        
        // sailServoPosition ¥ rudderServoPosition - ASPireActuator
        // distanceToNextWaypoint, bearingToNextWaypoint - CourseDataMsg
        // desiredHeading - NavigationControlMsg
        // m_tack - NavigationControlMsg

        //create timestamp----
        std::string timestamp_str=SysClock::timeStampStr();
        timestamp_str+=".";
        timestamp_str+= std::to_string(SysClock::millis());
        //--------------------

        node->m_dbLogger.log(node->m_rudderCommand, node->m_sailCommand, 
            0, 0, node->m_distanceToNextWaypoint, node->m_bearingToNextWaypoint,
            node->m_desiredHeading, node->m_tack, node->m_starboard, node->m_nextWaypointId,
            node->m_trueWindDir, false, timestamp_str);

        node->m_lock.unlock();
    }
}

