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

#include "../MessageBus/Message.h"

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

class AISDataMsg : public Message {
   public:
    AISDataMsg(NodeID destinationID,
               NodeID sourceID,
               std::vector<AISVessel>(vesselList),
               std::vector<AISVesselInfo>(infoList),
               float posLat,
               float posLon)
        : Message(MessageType::AISData, sourceID, destinationID),
          m_VesselList(vesselList),
          m_VesselInfoList(infoList),
          m_PosLat(posLat),
          m_PosLon(posLon) {}

    AISDataMsg(std::vector<AISVessel>(vesselList),
               std::vector<AISVesselInfo>(infoList),
               float posLat,
               float posLon)
        : Message(MessageType::AISData, NodeID::None, NodeID::None),
          m_VesselList(vesselList),
          m_VesselInfoList(infoList),
          m_PosLat(posLat),
          m_PosLon(posLon) {}

    AISDataMsg(MessageDeserialiser deserialiser) : Message(deserialiser) {
        union f_tag {
            uint8_t b[4];
            float fval;
        } union_float;

        union d_tag {
            uint8_t b[8];
            double fval;
        } union_double;

        union i_tag {
            uint8_t b[4];
            uint32_t fval;
        } union_int;

        AISVessel vessel;
        // TypeConst is the constant size of the messagetype
        int ves = 0, typeConst = 3;
        while (deserialiser.size() > ves + typeConst) {
            if (true) {
                union_int.b[0] = deserialiser.data()[typeConst + ves];
                union_int.b[1] = deserialiser.data()[typeConst + ves + 1];
                union_int.b[2] = deserialiser.data()[typeConst + ves + 2];
                union_int.b[3] = deserialiser.data()[typeConst + ves + 3];
                vessel.MMSI = union_int.fval;

                union_float.b[0] = deserialiser.data()[typeConst + ves + 4];
                union_float.b[1] = deserialiser.data()[typeConst + ves + 5];
                union_float.b[2] = deserialiser.data()[typeConst + ves + 6];
                union_float.b[3] = deserialiser.data()[typeConst + ves + 7];
                vessel.COG = union_float.fval;

                union_float.b[0] = deserialiser.data()[typeConst + ves + 8];
                union_float.b[1] = deserialiser.data()[typeConst + ves + 9];
                union_float.b[2] = deserialiser.data()[typeConst + ves + 10];
                union_float.b[3] = deserialiser.data()[typeConst + ves + 11];
                vessel.SOG = union_float.fval;

                union_double.b[0] = deserialiser.data()[typeConst + ves + 12];
                union_double.b[1] = deserialiser.data()[typeConst + ves + 13];
                union_double.b[2] = deserialiser.data()[typeConst + ves + 14];
                union_double.b[3] = deserialiser.data()[typeConst + ves + 15];
                union_double.b[4] = deserialiser.data()[typeConst + ves + 16];
                union_double.b[5] = deserialiser.data()[typeConst + ves + 17];
                union_double.b[6] = deserialiser.data()[typeConst + ves + 18];
                union_double.b[7] = deserialiser.data()[typeConst + ves + 19];
                vessel.latitude = union_double.fval;

                union_double.b[0] = deserialiser.data()[typeConst + ves + 20];
                union_double.b[1] = deserialiser.data()[typeConst + ves + 21];
                union_double.b[2] = deserialiser.data()[typeConst + ves + 22];
                union_double.b[3] = deserialiser.data()[typeConst + ves + 23];
                union_double.b[4] = deserialiser.data()[typeConst + ves + 24];
                union_double.b[5] = deserialiser.data()[typeConst + ves + 25];
                union_double.b[6] = deserialiser.data()[typeConst + ves + 26];
                union_double.b[7] = deserialiser.data()[typeConst + ves + 27];
                vessel.longitude = union_double.fval;

                m_VesselList.push_back(vessel);
            } else {
                m_valid = false;
            }
            // Add the size of the struct AISVessel [bytes]
            ves += 28;
        }
    }

    virtual ~AISDataMsg() {}

    std::vector<AISVessel> vesselList() { return m_VesselList; }
    std::vector<AISVesselInfo> vesselInfoList() { return m_VesselInfoList; }
    uint32_t MMSI(int vessel) { return m_VesselList[vessel].MMSI; }
    double latitude(int vessel) { return m_VesselList[vessel].latitude; }
    double longitude(int vessel) { return m_VesselList[vessel].longitude; }
    float COG(int vessel) { return m_VesselList[vessel].COG; }
    float SOG(int vessel) { return m_VesselList[vessel].SOG; }
    float posLat() const { return m_PosLat; }
    float posLon() const { return m_PosLon; }

    ///----------------------------------------------------------------------------------
    /// Serialises the message into a MessageSerialiser
    ///----------------------------------------------------------------------------------
    virtual void Serialise(MessageSerialiser& serialiser) const {
        Message::Serialise(serialiser);
        if (m_VesselList.size() == 0) {
            return;
        }
        for (auto& vessel : m_VesselList) {
            serialiser.serialise(vessel.MMSI);
            serialiser.serialise(vessel.COG);
            serialiser.serialise(vessel.SOG);
            serialiser.serialise(vessel.latitude);
            serialiser.serialise(vessel.longitude);
        }
        for (auto& info : m_VesselInfoList) {
            serialiser.serialise(info.MMSI);
            serialiser.serialise(info.length);
            serialiser.serialise(info.beam);
        }
        serialiser.serialise(m_PosLat);
        serialiser.serialise(m_PosLon);
    }

   private:
    std::vector<AISVessel> m_VesselList;
    std::vector<AISVesselInfo> m_VesselInfoList;
    double m_PosLat;
    double m_PosLon;
};
