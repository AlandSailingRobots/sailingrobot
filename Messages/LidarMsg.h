/****************************************************************************************
 *
 * File:
 * 		LidarMsg
 *
 * Purpose:
 *		Contains a distance in cm
 *
 * Developer Notes:
 *
 ***************************************************************************************/

#pragma once

#include "Message.h"

class LidarMsg : public Message {
public:
	LidarMsg(	NodeID destinationID, NodeID sourceID,int distance )

		:Message(MessageType::LidarData, sourceID, destinationID),m_distance(distance)
	{ }

    LidarMsg(NodeID sourceID, int distance)

		:Message(MessageType::LidarData,sourceID, NodeID::None), m_distance(distance)
	{ }

	LidarMsg(int distance)

		:Message(MessageType::LidarData, NodeID::None, NodeID::None), m_distance(distance)
	{ }

	virtual ~LidarMsg() { }

	int distance { return m_distance; }

private:
	int	m_distance;
};

