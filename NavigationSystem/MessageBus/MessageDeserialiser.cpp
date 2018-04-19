/****************************************************************************************
 *
 * File:
 * 		MessageDeserialiser.cpp
 *
 * Purpose:
 *		Deserialises a message into a block of bytes.
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/


#include "MessageBus/MessageDeserialiser.h"
#include <cstring>


MessageDeserialiser::MessageDeserialiser(uint8_t* data, uint8_t size)
	:m_data(data), m_index(0), m_size(size)
{

}

MessageDeserialiser::~MessageDeserialiser()
{

}

bool MessageDeserialiser::readUint8_t(uint8_t& data)
{
	if(m_index + sizeof(uint8_t) <= m_size)
	{

		data = m_data[m_index];
		m_index++;
		return true;
	}

	return false;
}

bool MessageDeserialiser::readUint16_t(uint16_t& data)
{
	if(m_index + sizeof(uint16_t) <= m_size)
	{
		data = 0;
		for(unsigned int i = 0; i < sizeof(uint16_t); i++)
		{
			data = data | (m_data[m_index] << (8 * (i)));
			m_index++;

		}

		return true;
	}

	return false;
}

bool MessageDeserialiser::readUint32_t(uint32_t& data)
{
	if(m_index + sizeof(uint32_t) <= m_size)
	{
		data = 0;
		for(unsigned int i = 0; i < sizeof(uint32_t); i++)
		{
			data = data | (m_data[m_index] << (8 * (i)));
			m_index++;

		}

		return true;
	}

	return false;
}

bool MessageDeserialiser::readInt(int& data)
{
	if(m_index + sizeof(int) <= m_size)
	{
		data = 0;
		for(unsigned int i = 0; i < sizeof(int); i++)
		{
			data = data | (m_data[m_index] << (8 * (i)));
			m_index++;

		}

		return true;
	}

	return false;
}

bool MessageDeserialiser::readFloat(float& data)
{
	if(m_index + sizeof(float) <= m_size)
	{
		data = 0;

		memcpy(&data, m_data + m_index, sizeof(float));
		m_index+= sizeof(float);

		return true;
	}

	return false;
}

bool MessageDeserialiser::readDouble(double& data)
{
	if(m_index + sizeof(double) <= m_size)
	{
		data = 0;
		memcpy(&data, m_data + m_index, sizeof(double));
		m_index+= sizeof(double);

		return true;
	}

	return false;
}

bool MessageDeserialiser::readBool(bool& data)
{
	if(m_index + sizeof(data) <= m_size)
	{
		data = m_data[m_index];
		m_index++;
		return true;
	}

	return false;
}

bool MessageDeserialiser::readMat(cv2::Mat& data)
{
	if(m_index + sizeof(data) <= m_size)
	{
		data = m_data[m_index];
		m_index++;
		return true;
	}

	return false;
}

bool MessageDeserialiser::readMessageType(MessageType& data)
{
	if(m_index + sizeof(data) <= m_size)
	{
		data = (MessageType)m_data[m_index];
		m_index++;
		return true;
	}

	return false;
}

bool MessageDeserialiser::readNodeID(NodeID& data)
{
	if(m_index + sizeof(data) <= m_size)
	{
		data = (NodeID)m_data[m_index];
		m_index++;
		return true;
	}

	return false;
}

uint8_t MessageDeserialiser::size()
{
	return m_size;
}

uint8_t* MessageDeserialiser::data()
{
	return m_data;
}
