/****************************************************************************************
 *
 * File:
 * 		AISDataMsg.h
 *
 * Purpose:
 *		An AISDataMsg contains the nearby vessels found by the AIS
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/

#pragma once

#include "MessageBus/Message.h"


class AISDataMsg : public Message {
public:
	AISDataMsg(NodeID destinationID, NodeID sourceID, std::vector<AISVessel>(vesselList))
		:Message(MessageType::SolarData, sourceID, destinationID), m_VesselList(vesselList)
	{ }

	AISDataMsg(std::vector<AISVessel>(vesselList))
		:Message(MessageType::SolarData, NodeID::None, NodeID::None), m_VesselList(vesselList)
	{ }

	AISDataMsg(MessageDeserialiser deserialiser)
		:Message(deserialiser)
	{
		for (auto &vessel: m_VesselList) {
			if (!deserialiser.readUint16_t(vessel->COG) ||
				!deserialiser.readUint16_t(vessel->SOG) ||
				!deserialiser.readInt(vessel->MMSI) ||
				!deserialiser.readDouble(vessel->latitude) ||
				!deserialiser.readDouble(vessel->longitude)) {
				m_valid = false;
			}
		}
	}

	virtual ~AISDataMsg() { }

	std::vector<AISVessel> vesselList() { return m_VesselList; }

  ///----------------------------------------------------------------------------------
	/// Serialises the message into a MessageSerialiser
	///----------------------------------------------------------------------------------
	virtual void Serialise(MessageSerialiser& serialiser) const
	{
		Message::Serialise(serialiser);

		for (auto &vessel: m_VesselList) {
			serialiser.serialise(vessel->COG);
			serialiser.serialise(vessel->SOG);
			serialiser.serialise(vessel->MMSI);
			serialiser.serialise(vessel->latitude);
			serialiser.serialise(vessel->longitude);
		}
	}

private:
	std::vector<AISVessel> m_VesselList;
};
