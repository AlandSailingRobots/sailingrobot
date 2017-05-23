#pragma once

#include "Nodes/ActiveNode.h"
#include "Messages/WindStateMsg.h"
#include "Messages/NavigationControlMsg.h"
#include "Messages/MessageTypes.h"
#include "MessageBus/MessageBus.h"
#include "dbhandler/DBLogger.h"

#include <mutex>
#include <iostream>

class DBLoggerNode: public ActiveNode {
public:
    DBLoggerNode(MessageBus& msgBus, DBHandler& db);
    ~DBLoggerNode(){};

    void processMessage(const Message* message);

    void start();

private:

    static void DBLoggerNodeThreadFunc(ActiveNode* nodePtr);

    DBHandler &m_db;
	DBLogger m_dbLogger;

    int m_TimeBetweenMsgs;
 
    std::mutex m_lock;

};