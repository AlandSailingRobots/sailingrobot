//
//	Wind State Message containing True Wind Speed & Direction,
//   as well as Apparent Wind Speed & Direction
//

#pragma once

#include "Messages/Message.h"
#include "Messages/MessageTypes.h"
#include "Nodes/NodeIDs.h"

class WindStateMsg : public Message {
public:
	WindStateMsg(NodeID sourceID, NodeID destinationID, double trueWindSpeed,
					double trueWindDir, double ApparentWindSpeed, double ApparentWindDir)
	: Message(MessageType::WindState, sourceID, destinationID), m_trueWindSpeed(trueWindSpeed),
		m_trueWindDir(trueWindDir), m_apparentWindSpeed(ApparentWindSpeed), m_apparentWindDir(ApparentWindDir)
	{  }

	WindStateMsg(double trueWindSpeed, double trueWindDir, double ApparentWindSpeed, double ApparentWindDir)
	: Message(MessageType::WindState, NodeID::None, NodeID::None), m_trueWindSpeed(trueWindSpeed),
		m_trueWindDir(trueWindDir), m_apparentWindSpeed(ApparentWindSpeed), m_apparentWindDir(ApparentWindDir)
	{  }

	WindStateMsg(MessageDeserialiser deserialiser)
	: Message(deserialiser)
	{
		if(!deserialiser.readDouble(m_trueWindSpeed) 	 ||
		   !deserialiser.readDouble(m_trueWindDir)   	 ||
		   !deserialiser.readDouble(m_apparentWindSpeed) ||	
		   !deserialiser.readDouble(m_apparentWindDir))
		{
			m_valid = false;
		}
	}

	virtual ~WindStateMsg() { }

	double trueWindSpeed()     		{ return m_trueWindSpeed; }
	double trueWindDirection() 		{ return m_trueWindDir;   }
	double apparentWindSpeed() 		{ return m_apparentWindSpeed; }
	double apparentWindDirection()  { return m_apparentWindDir;   }

private:
	double m_trueWindSpeed;
	double m_trueWindDir;
	double m_apparentWindSpeed;
	double m_apparentWindDir;
};