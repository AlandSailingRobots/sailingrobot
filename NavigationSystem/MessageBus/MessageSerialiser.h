/****************************************************************************************
 *
 * File:
 * 		MessageSerialiser.h
 *
 * Purpose:
 *		Serialises a message into a block of bytes.
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/

#pragma once

#include <stdint.h>
#include <string>
#include "../MessageBus/MessageTypes.h"
#include "../MessageBus/NodeIDs.h"

#define MAX_MESSAGE_SIZE 256

class MessageSerialiser {
   public:
    MessageSerialiser() : m_ptr(0) {}

    ///----------------------------------------------------------------------------------
    /// Serialises the data into a byte buffer.
    ///----------------------------------------------------------------------------------
    void serialise(uint8_t data);
    void serialise(uint16_t data);
    void serialise(uint32_t data);
    void serialise(uint64_t data);
    void serialise(int data);
    void serialise(float data);
    void serialise(double data);
    void serialise(bool data);
    void serialise(MessageType data);
    void serialise(NodeID data);
    void serialise(uint8_t* data, uint8_t size);

    ///----------------------------------------------------------------------------------
    /// Returns a pointer to the data.
    ///----------------------------------------------------------------------------------
    uint8_t* data() { return m_data; }

    ///----------------------------------------------------------------------------------
    /// Returns the number of bytes the data takes up.
    ///----------------------------------------------------------------------------------
    uint8_t size() { return m_ptr; }

   private:
    uint8_t m_data[MAX_MESSAGE_SIZE];
    uint8_t m_ptr;
};
