/****************************************************************************************
 *
 * File:
 * 		ObstacleVectorMsg
 *
 * Purpose:
 *		Contains a vector with the angular position and dist of all the detected obstacles
 *
 * Developer Notes:
 *
 ***************************************************************************************/

#pragma once

#include "Message.h"


struct ObstacleData {
       double minDistanceToObstacle;
       double maxDistanceToObstacle; // -1 = infinite
       double LeftBoundheadingRelativeToBoat;
       double RightBoundheadingRelativeToBoat;
   };



class ObstacleVectorMsg : public Message {
public:
	ObstacleVectorMsg(	NodeID destinationID, NodeID sourceID,std::vector<ObstacleData> obstacles )

		:Message(MessageType::ObstacleVector, sourceID, destinationID),m_obstacles(obstacles)
	{ }

    ObstacleVectorMsg(NodeID sourceID,std::vector<ObstacleData> obstacles)

		:Message(MessageType::ObstacleVector,sourceID, NodeID::None), m_obstacles(obstacles)
	{ }

	ObstacleVectorMsg(std::vector<ObstacleData> obstacles)

		:Message(MessageType::ObstacleVector, NodeID::None, NodeID::None), m_obstacles(obstacles)
	{ }

	virtual ~ObstacleVectorMsg() { }

	std::vector<ObstacleData> obstacles() { return m_obstacles; }

private:
	std::vector<ObstacleData> m_obstacles;
};
