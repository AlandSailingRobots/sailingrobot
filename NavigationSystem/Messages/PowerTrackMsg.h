/****************************************************************************************
 *
 * File:
 * 		PowerTrackMsg.h
 *
 * Purpose:
 *		A PowerTrackMsg contains the power state of the vessel at a given time.
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/

#ifndef NAVIGATIONSYSTEM_POWERTRACKMSG_H
#define NAVIGATIONSYSTEM_POWERTRACKMSG_H

#include "../MessageBus/Message.h"

 class PowerTrackMsg : public Message {
   public:
   	PowerTrackMsg(NodeID destinationID,
                  NodeID sourceID,
                  float balance)
   	: Message(MessageType::PowerTrack, sourceID, destinationID),
      m_PowerBalance(balance) {}

   	PowerTrackMsg(float balance)
   	: Message(MessageType::PowerTrack, NodeID::None, NodeID::None),
      m_PowerBalance(balance) {}

   	PowerTrackMsg(MessageDeserialiser deserialiser) : Message(deserialiser) {
       if (!deserialiser.readFloat(m_PowerBalance)) {
	        m_valid = false;
        }
   	}

    virtual void Serialise(MessageSerialiser& serialiser) const {
      Message::Serialise(serialiser);
      serialiser.serialise(m_PowerBalance);
    }

   	virtual ~PowerTrackMsg() {}

    float getBalance() const { return m_PowerBalance; }

   	///---------------------------------------------------------------------------
    /// Serialises the message into a MessageSerialiser
    ///---------------------------------------------------------------------------

  private:
    float m_PowerBalance;
 };

#endif //NAVIGATIONSYSTEM_POWERTRACKMSG_H