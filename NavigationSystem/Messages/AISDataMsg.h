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

#include <vector>

class AISDataMsg : public Message {
public:
	AISDataMsg(NodeID destinationID, NodeID sourceID, std::vector<AISVessel>(vesselList), float posLat, float posLon)
		:Message(MessageType::AISData, sourceID, destinationID), m_VesselList(vesselList), m_PosLat(posLat), m_PosLon(posLon)
	{ }

	AISDataMsg(std::vector<AISVessel>(vesselList), float posLat, float posLon)
		:Message(MessageType::AISData, NodeID::None, NodeID::None), m_VesselList(vesselList), m_PosLat(posLat), m_PosLon(posLon)
	{ }

	AISDataMsg(MessageDeserialiser deserialiser)
		:Message(deserialiser)
	{

    union f_tag {
       uint8_t b[4];
       float fval;
    } union_float;

    union i_tag {
       uint8_t b[4];
       uint32_t fval;
    } union_int;

		AISVessel vessel;
		// TypeConst is the constant size of the messagetype
		int ves = 0, typeConst = 3;
		while (deserialiser.size() > ves+typeConst) {
			if (true) {
				vessel.COG = combuint8(deserialiser.data()[typeConst+ves], deserialiser.data()[typeConst+ves+1]);

				vessel.SOG = combuint8(deserialiser.data()[typeConst+ves+2], deserialiser.data()[typeConst+ves+3]);

				union_int.b[0] = deserialiser.data()[typeConst+ves+4];
				union_int.b[1] = deserialiser.data()[typeConst+ves+5];
				union_int.b[2] = deserialiser.data()[typeConst+ves+6];
				union_int.b[3] = deserialiser.data()[typeConst+ves+7];
				vessel.MMSI = union_int.fval;

				union_float.b[0] = deserialiser.data()[typeConst+ves+8];
				union_float.b[1] = deserialiser.data()[typeConst+ves+9];
				union_float.b[2] = deserialiser.data()[typeConst+ves+10];
				union_float.b[3] = deserialiser.data()[typeConst+ves+11];
				vessel.latitude = union_float.fval;

				union_float.b[0] = deserialiser.data()[typeConst+ves+12];
				union_float.b[1] = deserialiser.data()[typeConst+ves+13];
				union_float.b[2] = deserialiser.data()[typeConst+ves+14];
				union_float.b[3] = deserialiser.data()[typeConst+ves+15];
				vessel.longitude = union_float.fval;
				m_VesselList.push_back(vessel);
			}
			else {
				m_valid = false;
			}
			// Add the size of the struct AISVessel
			ves += 16;
		}
	}

	virtual ~AISDataMsg() { }

	std::vector<AISVessel> vesselList() { return m_VesselList; }
	uint32_t MMSI(int vessel) { return m_VesselList[vessel].MMSI; }
	float latitude(int vessel) { return m_VesselList[vessel].latitude; }
	float longitude(int vessel) { return m_VesselList[vessel].longitude; }
	uint16_t COG(int vessel) { return m_VesselList[vessel].COG; }
	uint16_t SOG(int vessel) { return m_VesselList[vessel].SOG; }
	float posLat() { return m_PosLat; }
	float posLon() { return m_PosLon; }

  ///----------------------------------------------------------------------------------
	/// Serialises the message into a MessageSerialiser
	///----------------------------------------------------------------------------------
	virtual void Serialise(MessageSerialiser& serialiser) const
	{
		Message::Serialise(serialiser);
		if (m_VesselList.size()==0) {
			return;
		}
		for (auto &vessel: m_VesselList) {
			serialiser.serialise(vessel.COG);
			serialiser.serialise(vessel.SOG);
			serialiser.serialise(vessel.MMSI);
			serialiser.serialise(vessel.latitude);
			serialiser.serialise(vessel.longitude);
		}
		serialiser.serialise(m_PosLat);
		serialiser.serialise(m_PosLon);
	}

private:
	uint16_t combuint8(uint8_t lower, uint8_t higher) {
		return ((uint16_t) higher << 8) | lower;
	}

	std::vector<AISVessel> m_VesselList;
	float m_PosLat;
	float m_PosLon;
};
