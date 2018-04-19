/****************************************************************************************
 *
 * File:
 * 		ThermalImagerDataMsg.h
 *
 * Purpose:
 *		A ThermalImagerDataMsg contains the current frame seen by the camera
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/

#pragma once

#include "MessageBus/Message.h"
#include <opencv2/core.hpp>

#include <vector>

struct AISVessel {
  uint32_t MMSI;
  float COG;
  float SOG;
  double latitude;
  double longitude;
};

struct AISVesselInfo {
  uint32_t MMSI;
  float length;
  float beam;
};

class ThermalImagerDataMsg : public Message {
public:
	ThermalImagerDataMsg(NodeID destinationID, NodeID sourceID, cv2::Mat frame)
		:Message(MessageType::ThermalImagerData, sourceID, destinationID), m_Frame(frame)
	{ }

	ThermalImagerDataMsg(std::vector<AISVessel>(vesselList), cv2::Mat frame)
		:Message(MessageType::ThermalImagerData, NodeID::None, NodeID::None), m_Frame(frame)
	{ }

	ThermalImagerDataMsg(MessageDeserialiser deserialiser)
		:Message(deserialiser)
	{
    	if(	!deserialiser.readMat(m_Frame))
        {
            m_valid = false;
        }
	}

	virtual ~ThermalImagerDataMsg() { }

	cv2::Mat frame() const { return m_Frame; }

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
			serialiser.serialise(vessel.MMSI);
			serialiser.serialise(vessel.COG);
			serialiser.serialise(vessel.SOG);
			serialiser.serialise(vessel.latitude);
			serialiser.serialise(vessel.longitude);
		}
		for (auto& info: m_VesselInfoList) {
			serialiser.serialise(info.MMSI);
			serialiser.serialise(info.length);
			serialiser.serialise(info.beam);
		}
		serialiser.serialise(m_PosLat);
		serialiser.serialise(m_PosLon);
	}

private:
	cv2::Mat m_Frame;
};
