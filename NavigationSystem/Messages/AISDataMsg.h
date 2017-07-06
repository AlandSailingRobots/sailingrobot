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

typedef unsigned char uchar;

union f_tag {
   uint8_t b[4];
   float fval;
} uf;

union i_tag {
   uint8_t b[4];
   int fval;
} ui;

struct AISVessel {
  uint16_t COG;
  uint16_t SOG;
  int MMSI;
  float latitude;
  float longitude;
};

class AISDataMsg : public Message {
public:
	AISDataMsg(NodeID destinationID, NodeID sourceID, std::vector<AISVessel>(vesselList))
		:Message(MessageType::AISData, sourceID, destinationID), m_VesselList(vesselList)
	{ }

	AISDataMsg(std::vector<AISVessel>(vesselList))
		:Message(MessageType::AISData, NodeID::None, NodeID::None), m_VesselList(vesselList)
	{ }

	AISDataMsg(MessageDeserialiser deserialiser)
		:Message(deserialiser)
	{

		AISVessel vessel;
		int ves = 0, typeConst = 3;
		while (deserialiser.size() > ves+typeConst) {
			if (true) {
				vessel.COG = combuint8(deserialiser.data()[typeConst+ves], deserialiser.data()[typeConst+ves+1]);

				vessel.SOG = combuint8(deserialiser.data()[typeConst+ves+2], deserialiser.data()[typeConst+ves+3]);

				ui.b[0] = deserialiser.data()[typeConst+ves+4];
				ui.b[1] = deserialiser.data()[typeConst+ves+5];
				ui.b[2] = deserialiser.data()[typeConst+ves+6];
				ui.b[3] = deserialiser.data()[typeConst+ves+7];
				vessel.MMSI = ui.fval;

				uf.b[0] = deserialiser.data()[typeConst+ves+8];
				uf.b[1] = deserialiser.data()[typeConst+ves+9];
				uf.b[2] = deserialiser.data()[typeConst+ves+10];
				uf.b[3] = deserialiser.data()[typeConst+ves+11];
				vessel.latitude = uf.fval;

				uf.b[0] = deserialiser.data()[typeConst+ves+12];
				uf.b[1] = deserialiser.data()[typeConst+ves+13];
				uf.b[2] = deserialiser.data()[typeConst+ves+14];
				uf.b[3] = deserialiser.data()[typeConst+ves+15];
				vessel.longitude = uf.fval;
				m_VesselList.push_back(vessel);
			}
			else {
				m_valid = false;
			}
			ves += 16;
		}
	}

	virtual ~AISDataMsg() { }

	std::vector<AISVessel> vesselList() { return m_VesselList; }
	int MMSI(int vessel) { return m_VesselList[vessel].MMSI; }
	float latitude(int vessel) { return m_VesselList[vessel].latitude; }
	float longitude(int vessel) { return m_VesselList[vessel].longitude; }
	uint16_t COG(int vessel) { return m_VesselList[vessel].COG; }
	uint16_t SOG(int vessel) { return m_VesselList[vessel].SOG; }

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
	}

private:
	uint16_t combuint8(uint8_t lower, uint8_t higher) {
		return ((uint16_t) higher << 8) | lower;
	}

	int combuint8_int(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3) {
		int i;
		uchar b[] = {b3, b2, b1, b0};
		memcpy(&i, &b, sizeof(i));
		return i;
	}

	float combuint8_float(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3) {
		float f;
		uchar b[] = {b3, b2, b1, b0};
		memcpy(&f, &b, sizeof(f));
		return f;
	}

	std::vector<AISVessel> m_VesselList;
};
