/****************************************************************************************
 *
 * File:
 * 		DataLink.h
 *
 * Purpose:
 *		A data link interacts with the transport layer, it deals with sending and receiving
 *		network frames over the hardware or via a operating system.
 *
 * Developer Notes:
 *
 ***************************************************************************************/

#pragma once

#include <stdint.h>
#include <string>
#include "../Network/NetworkFrame.h"

class DataLink {
   public:
    DataLink(bool useSlip) : m_initialised(false), m_maxFrameSize(0), m_useSlip(useSlip) {}

    virtual ~DataLink() {}

    ///----------------------------------------------------------------------------------
    /// Initialises the data link. This function should ensure that the hardware layer
    /// is up and functioning.
    ///----------------------------------------------------------------------------------
    virtual bool initialise(uint16_t frameSize) {
        m_maxFrameSize = frameSize;
        return true;
    }

    ///----------------------------------------------------------------------------------
    /// Transmits a network frame across the data link.
    ///----------------------------------------------------------------------------------
    void transmit(const NetworkFrame& frame);

    ///----------------------------------------------------------------------------------
    /// Attempts to read a network frame from over the data link, if a NetworkFrame was
    /// successfully read then the function files out frame and returns true.
    ///----------------------------------------------------------------------------------
    bool receive(NetworkFrame& frame);

    virtual std::string sendATCommand(std::string command, uint16_t responseSize) = 0;

   protected:
    uint16_t maxFrameSize() const { return m_maxFrameSize; }

    ///----------------------------------------------------------------------------------
    /// Listens for a frame start character, returns true if there is one.
    ///----------------------------------------------------------------------------------
    bool foundFrameStart();

    virtual void writeData(const uint8_t* data, uint8_t size) = 0;
    virtual void readData(uint8_t* data, uint8_t size) = 0;
    virtual int readByte() = 0;
    virtual void writeByte(uint8_t byte) = 0;
    virtual bool dataAvailable() = 0;

    bool m_initialised;

   private:
    uint16_t m_maxFrameSize;
    bool m_useSlip;
};
