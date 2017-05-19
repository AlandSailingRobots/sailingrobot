#include "DBLoggerNode.h"

void DBLoggerNode::processMessage(const Message* msg)
{
    static int i = 0;
    MessageType type = msg->messageType();

    switch(type) 
    {
        case MessageType::Windstate: 
        {
            const WindStateMsg* windStateMsg = static_cast<const WindStateMsg*>(msg);
            m_trueWindSpeed = windStateMsg->trueWindSpeed();
            m_trueWindDir = windStateMsg->trueWindDirection();
            m_apparentWindSpeed = windStateMsg->apparentWindSpeed();
            m_apparentWindDir = windStateMsg->apparentWindDirection();
        }
        break;
        default:
        return;
    }
}