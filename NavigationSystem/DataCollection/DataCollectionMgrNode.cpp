


void  DataCollectionMgrNode::processMessage(const Message* msg) {

	MessageType type = msg->messageType();
    
	if(type == MessageType::WayPointDataMsg) {

	}
    else if(type == MessageType::LocalConfigChangeMsg) {

    }

    // send DataCollectionStartMsg, DataCollectionStopMsg and DataRequestMsg
    // CANMarineSensorTransmissionNode
}
