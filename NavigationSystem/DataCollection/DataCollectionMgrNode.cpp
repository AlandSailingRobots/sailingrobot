#include <memory>
#include "../DataCollection/DataCollectionMgrNode.h"
#include "../Messages/WaypointDataMsg.h"
#include "../Messages/DataCollectionStopMsg.h"
#include "../Messages/DataCollectionStartMsg.h"
#include "../Messages/DataRequestMsg.h"

DataCollectionMgrNode::DataCollectionMgrNode(MessageBus &msgBus, DBHandler& db) :
										Node(NodeID::DataCollectionMgr, msgBus), m_db(db) {

    msgBus.registerNode(*this, MessageType::WaypointData);
    msgBus.registerNode(*this, MessageType::LocalConfigChange);
}


void DataCollectionMgrNode::processMessage(const Message* msg) {

	MessageType type = msg->messageType();

	if(type == MessageType::WaypointData) {
		const WaypointDataMsg * waypoint = dynamic_cast<const WaypointDataMsg*>(msg);

		if(m_measureAtCheckpoint && waypoint->isCheckpoint()) {
			sendRequestMessage();
		}
	}
    else if(type == MessageType::LocalConfigChange) {
		readConfig();
		sendIntervalMessage();
    }

}

void DataCollectionMgrNode::readConfig() {
	m_timeInterval = m_db.tableColumnValueInt("config_marine_sensors", "time_interval");
	m_measureAtCheckpoint = m_db.tableColumnValueInt("config_marine_sensors", "measure_at_checkpoint");
}

void DataCollectionMgrNode::sendIntervalMessage() {
	MessagePtr msg;
	if(m_timeInterval == 0) {
		msg = std::make_unique<DataCollectionStopMsg>();
	}
	else {
		msg = std::make_unique<DataCollectionStartMsg>(m_timeInterval);
	}

    m_MsgBus.sendMessage(std::move(msg));
}

void DataCollectionMgrNode::sendRequestMessage() {
	MessagePtr msg = std::make_unique<DataRequestMsg>();
	m_MsgBus.sendMessage(std::move(msg));
}
