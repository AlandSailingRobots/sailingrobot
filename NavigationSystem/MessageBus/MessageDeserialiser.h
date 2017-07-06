/****************************************************************************************
 *
 * File:
 * 		MessageDeserialiser.h
 *
 * Purpose:
 *		Deserialises a message from a block of bytes.
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/


#pragma once


#include <stdint.h>
#include "MessageBus/MessageTypes.h"
#include "MessageBus/NodeIDs.h"


class MessageDeserialiser {
public:
	MessageDeserialiser(uint8_t* data, uint8_t size);
	~MessageDeserialiser();

	bool readUint8_t(uint8_t& data);
	bool readUint16_t(uint16_t& data);
	bool readUint32_t(uint32_t& data);
	bool readInt(int& data);
	bool readFloat(float& data);
	bool readDouble(double& data);
	bool readBool(bool& data);
	bool readMessageType(MessageType& data);
	bool readNodeID(NodeID& data);

	void resetInternalPtr() { m_index = 0;}

	uint8_t* data();
	uint8_t index();
	uint8_t size();

private:
	uint8_t* 	m_data;
	uint8_t		m_index;
	uint8_t		m_size;
};
