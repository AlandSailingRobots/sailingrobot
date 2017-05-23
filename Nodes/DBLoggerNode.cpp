#include "DBLoggerNode.h"

#include "Messages/WindStateMsg.h"
#include "SystemServices/Timer.h"


DBLoggerNode::DBLoggerNode(MessageBus& msgBus, DBHandler& db)
: ActiveNode(NodeID::DBLoggerNode, msgBus), m_db(db), m_dbLogger(5, db)
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
            double m_trueWindSpeed = windStateMsg->trueWindSpeed();
            m_trueWindSpeed = m_trueWindSpeed;
            /*m_trueWindDir = windStateMsg->trueWindDirection();
            m_apparentWindSpeed = windStateMsg->apparentWindSpeed();
            m_apparentWindDir = windStateMsg->apparentWindDirection(); */
        }
        break;
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
        // msg - WindStateMsg
        // rudderCommand_norm & sailCommand_norm - ActuatorPositionMsg
        // distanceToNextWaypoint, bearingToNextWaypoint - CourseDataMsg
        //m_dbLogger.log(msg, rudderCommand_norm, sailCommand_norm, 0, 0, distanceToNextWaypoint, bearingToNextWaypoint, desiredHeading, m_tack, getGoingStarboard(), m_nextWaypointId, msg->trueWindDirection(), false, timestamp_str);

        node->m_lock.unlock();
    }
}

