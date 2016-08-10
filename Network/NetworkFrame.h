/****************************************************************************************
 *
 * File:
 * 		NetworkFrame.h
 *
 * Purpose:
 * 		A network frame is a self-garbage collecting buffer for data that is to be sent
 * 		across a network.
 *
 * Developer Notes:
 * 		The use of a internal reference counter allows us to know when to destroy this
 * 		class's data.
 *
 ***************************************************************************************/


#pragma once


#include <stdint.h>
#include <cstddef> /* NULL */


struct NetworkFrame
{
	///----------------------------------------------------------------------------------
	/// Constructs an empty network frame with no data or reference counter.
	///----------------------------------------------------------------------------------
	NetworkFrame()
		:m_data(NULL), m_size(0), m_references(NULL)
	{ }

	///----------------------------------------------------------------------------------
	/// Constructs a network frame with a block of data. This will also begin a reference
	/// counter on the data which will control when the frame is garbage collected.
	///----------------------------------------------------------------------------------
	NetworkFrame(uint8_t* data, uint16_t size)
		:m_data(data), m_size(size)
	{
		m_references = new uint8_t(1);
	}

	///----------------------------------------------------------------------------------
	/// Copies a network frame and its reference counter.
	///----------------------------------------------------------------------------------
	NetworkFrame(NetworkFrame& copy)
		:m_data(copy.m_data), m_size(copy.m_size), m_references(copy.m_references)
	{
		*(m_references) = (*m_references) + 1;
	}

	~NetworkFrame()
	{
		cleanup();
	}

	///----------------------------------------------------------------------------------
	/// Replaces the current data in the network frame, this will decrement its current
	/// reference pointer and replace it
	///----------------------------------------------------------------------------------
	void setData(uint8_t* data, uint16_t size)
	{
		cleanup();

		m_references = new uint8_t(1);
		m_data = data;
		m_size = size;
	}

	uint8_t* m_data;
	uint16_t m_size;
private:
	void cleanup()
	{
		if(m_references != NULL)
		{
			*(m_references) = (*m_references) - 1;
			if(m_references == 0)
			{
				delete m_references;
				delete m_data;
				m_data = NULL;
			}
		}
	}

	uint8_t* m_references;
};
