/****************************************************************************************
 *
 * File:
 * 		MessageSerialiser.cpp
 *
 * Purpose:
 *		Serialises a message into a block of bytes.
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/


#include "MessageSerialiser.h"
#include "SystemServices/Logger.h"


void MessageSerialiser::serialise(uint8_t data)
{
	if(m_ptr != MAX_MESSAGE_SIZE)
	{
		m_data[m_ptr++] = data;
	}
	else
	{
		Logger::error("%s Message is full, failed to insert more data", __PRETTY_FUNCTION__);
	}
}

void MessageSerialiser::serialise(uint16_t data)
{
	uint8_t* ptr = (uint8_t*)&data;
	serialise(ptr, sizeof(data));
}

void MessageSerialiser::serialise(uint32_t data)
{
	uint8_t* ptr = (uint8_t*)&data;
	serialise(ptr, sizeof(data));
}

void MessageSerialiser::serialise(int data)
{
	uint8_t* ptr = (uint8_t*)&data;
	serialise(ptr, sizeof(data));
}

void MessageSerialiser::serialise(float data)
{
	uint8_t* ptr = (uint8_t*)&data;
	serialise(ptr, sizeof(data));
}

void MessageSerialiser::serialise(double data)
{
	uint8_t* ptr = (uint8_t*)&data;
	serialise(ptr, sizeof(data));
}

void MessageSerialiser::serialise(bool data)
{
	serialise((uint8_t)data);
}

void  MessageSerialiser::serialise(MessageType data)
{
	serialise((uint8_t)data);
}
void  MessageSerialiser::serialise(NodeID data)
{
	serialise((uint8_t)data);
}

void MessageSerialiser::serialise(uint8_t* data, uint8_t size)
{
	if(m_ptr != MAX_MESSAGE_SIZE)
	{
		for(unsigned int i = 0; i < size; i++)
		{
			m_data[m_ptr++] = data[i];
		}
	}
	else
	{
		Logger::error("%s Message is full, failed to insert more data", __PRETTY_FUNCTION__);
	}
}


