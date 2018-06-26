/****************************************************************************************
 *
 * File:
 * 		DataCollectionMgrNode.h
 *
 * Purpose:
 *		Listen to WayPointDataMsg and LocalConfigChangeMsg,
 *		send DataCollectionStartMsg, DataCollectionStopMsg and DataRequestMsg
 *		accordingly.
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/

#pragma once

#include "../Database/DBHandler.h"
#include "../MessageBus/MessageBus.h"

class DataCollectionMgrNode : public Node {
   public:
    DataCollectionMgrNode(MessageBus& msgBus, DBHandler& db);

    ///----------------------------------------------------------------------------------
    /// Process the message.
    ///----------------------------------------------------------------------------------
    void processMessage(const Message* msg);

    ///----------------------------------------------------------------------------------
    /// Read the new config data.
    ///----------------------------------------------------------------------------------
    void readConfig();

   private:
    void sendIntervalMessage();
    void sendRequestMessage();

    int m_timeInterval;
    bool m_measureAtCheckpoint;
    DBHandler& m_db;
};
