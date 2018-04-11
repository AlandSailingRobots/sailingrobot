

DataCollectionMgrNode::DataCollectionMgrNode(MessageBus &msgBus, DBHandler& db) :
										Node(NodeID::DataCollectionMgr, msgBus), m_db(db) {

    msgBus.registerNode(*this, MessageType::WayPointDataMsg);
    msgBus.registerNode(*this, MessageType::LocalConfigChangeMsg);
}


void DataCollectionMgrNode::processMessage(const Message* msg) {

	MessageType type = msg->messageType();

	if(type == MessageType::WayPointDataMsg) {
        if (readData(m_timeInterval, m_measureAtCheckpoint)) {

		}
	}
    else if(type == MessageType::LocalConfigChangeMsg) {

    }

    // send DataCollectionStartMsg, DataCollectionStopMsg and DataRequestMsg
    // CANMarineSensorTransmissionNode
}

bool DataCollectionMgrNode::readData(int& timeInterval, bool& measureAtCheckpoint) {
	if(m_db.getWaypointValues())
}

void DataCollectionMgr::sendMessage(int interval) {
	MessagePtr msg = std::make_unique<DataCollectionStartMsg>(interval);
    m_MsgBus.sendMessage(std::move(msg));
}
