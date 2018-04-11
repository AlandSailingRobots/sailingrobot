

DataCollectionMgrNode::DataCollectionMgrNode(MessageBus &msgBus) :
										Node(NodeID::DataCollectionMgr, msgBus), m_db(db) {

    msgBus.registerNode(*this, MessageType::WayPointDataMsg);
    msgBus.registerNode(*this, MessageType::LocalConfigChangeMsg);
}


void DataCollectionMgrNode::processMessage(const Message* msg) {

	MessageType type = msg->messageType();

	if(type == MessageType::WayPointDataMsg) {
		WayPointDataMsg * waypoint = dynamic_cast<WayPointDataMsg*>(&msg);
		if(m_measureAtCheckpoint && waypoint.isCheckpoint()) {
			sendRequestMessage();
		}
	}
    else if(type == MessageType::LocalConfigChangeMsg) {
		readConfig();
		sendIntervalMessage();
    }

}

bool DataCollectionMgrNode::readConfig() {
	m_timeInterval = m_db.retrieveCellAsInt("config_marine_sensors","1","time_interval");
	m_measureAtCheckpoint = m_db.retrieveCellAsInt("config_marine_sensors","1","measure_at_checkpoint");
}

void DataCollectionMgr::sendIntervalMessage() {
	if(m_timeInterval == 0) {
		MessagePtr msg = std::make_unique<DataCollectionStopMsg>();
	}
	else {
		MessagePtr msg = std::make_unique<DataCollectionStartMsg>(m_timeInterval);
	}

    m_MsgBus.sendMessage(std::move(msg));
}

void DataCollectionMgr::sendRequestMessage() {
	MessagePtr msg = std::make_unique<DataRequestMsg>(m_timeInterval);
	m_MsgBus.sendMessage(std::move(msg));
}
