

DataCollectionMgrNode::DataCollectionMgrNode(MessageBus &msgBus, DBHandler& db) :
										Node(NodeID::DataCollectionMgr, msgBus), m_db(db) {

    msgBus.registerNode(*this, MessageType::WayPointDataMsg);
    msgBus.registerNode(*this, MessageType::LocalConfigChangeMsg);
}


void DataCollectionMgrNode::processMessage(const Message* msg) {

	MessageType type = msg->messageType();

	if(type == MessageType::WayPointDataMsg) {

	}
    else if(type == MessageType::LocalConfigChangeMsg) {
		readConfig();
    }

}

bool DataCollectionMgrNode::readConfig() {
	m_timeInterval = m_db.retrieveCellAsInt("config_marine_sensors","1","time_interval");
	m_measureAtCheckpoint = m_db.retrieveCellAsInt("config_marine_sensors","1","measure_at_checkpoint");
}

void DataCollectionMgr::sendMessage(int interval) {
	MessagePtr msg = std::make_unique<DataCollectionStartMsg>(interval);
    m_MsgBus.sendMessage(std::move(msg));
}
