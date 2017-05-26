#pragma once

#include "dbhandler/DBLogger.h"
#include "Nodes/ActiveNode.h"
#include "Messages/MessageTypes.h"
#include "MessageBus/MessageBus.h"

#include <mutex>
#include <iostream>

class DBLoggerNode: public ActiveNode {
public:
    DBLoggerNode(MessageBus& msgBus, DBHandler& db, int TimeBetweenMsgs);

    void processMessage(const Message* message);

    void start();

    bool init();

private:

    static void DBLoggerNodeThreadFunc(ActiveNode* nodePtr);

    int DATA_OUT_OF_RANGE = -2000;

    DBHandler &m_db;
	DBLogger m_dbLogger;

	double m_trueWindDir            = DATA_OUT_OF_RANGE;

    double m_rudderCommand          = DATA_OUT_OF_RANGE;
    double m_sailCommand            = DATA_OUT_OF_RANGE;

    double m_distanceToNextWaypoint = DATA_OUT_OF_RANGE;
    double m_bearingToNextWaypoint  = DATA_OUT_OF_RANGE;

    double m_desiredHeading         = DATA_OUT_OF_RANGE;
    double m_tack                   = DATA_OUT_OF_RANGE;
    bool m_starboard;

    double m_nextWaypointId         = DATA_OUT_OF_RANGE;

    int m_TimeBetweenMsgs;
 
    std::mutex m_lock;

};