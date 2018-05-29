/****************************************************************************************
 *
 * File:
 * 		SLIP.h
 *
 * Purpose:
 *		Provides static functions for interacting with SLIP (Serial Line Internet Protocol)
 *
 * Developer Notes:
 *
 ***************************************************************************************/

#pragma once

#define SLIP_PACKET_START 0xC0
#define SLIP_PACKET_END 0xC1
#define SLIP_PACKET_ESCAPE 0xDB
#define SLIP_PACKET_ESCAPE_START 0xDC
#define SLIP_PACKET_ESCAPE_ESCAPE 0xDD
#define SLIP_PACKET_ESCAPE_END 0xDE

class SLIP {
   public:
    static bool isStartCharacter(uint8_t c) { return (c == SLIP_PACKET_START); }

    static bool isEndCharacter(uint8_t c) { return (c == SLIP_PACKET_END); }

    static bool isEscapeCharacter(uint8_t c) { return (c == SLIP_PACKET_ESCAPE); }

    static bool isSpecialCharacter(uint8_t c) {
        return (isStartCharacter(c) || isEndCharacter(c) || isEscapeCharacter(c));
    }

    static uint8_t getEscapedCharacter(uint8_t c) {
        switch (c) {
            case SLIP_PACKET_ESCAPE_START:
                return SLIP_PACKET_START;
            case SLIP_PACKET_ESCAPE_ESCAPE:
                return SLIP_PACKET_ESCAPE;
            case SLIP_PACKET_ESCAPE_END:
                return SLIP_PACKET_END;
            default:
                return c;
        }
    }

    static uint8_t getEscapeCharacter(uint8_t c) {
        switch (c) {
            case SLIP_PACKET_START:
                return SLIP_PACKET_ESCAPE_START;
            case SLIP_PACKET_ESCAPE:
                return SLIP_PACKET_ESCAPE_ESCAPE;
            case SLIP_PACKET_END:
                return SLIP_PACKET_ESCAPE_END;
            default:
                return c;
        }
    }

    static uint8_t packetCount(uint8_t* dataPtr, uint16_t dataLength, uint16_t maxPacketSize) {
        uint16_t currentPacketLength = 0;
        uint16_t packetCount = 1;

        for (uint16_t i = 0; i < dataLength; i++) {
            if (currentPacketLength == maxPacketSize) {
                currentPacketLength = 0;
                packetCount++;
            }

            if (isSpecialCharacter(dataPtr[i])) {
                currentPacketLength += 2;
            } else {
                currentPacketLength++;
            }
        }

        return packetCount;
    }

    ///----------------------------------------------------------------------------------
    /// Goes through a block of data counting how big a slip packet would be, this
    /// function returns the size of that slip packet. The third parameter is a limiter
    /// on the max possible size of a slip packet which will cause the function to return
    /// without inspecting all the bytes if this limit is hit. The remaining bytes left
    /// to process is set in the fourth and final parameter.
    ///----------------------------------------------------------------------------------
    static uint16_t splitSize(uint8_t* dataPtr,
                              uint16_t dataLength,
                              uint16_t maxPacketSize,
                              uint16_t& bytesLeftToProcess) {
        uint16_t currentPacketLength = 0;
        uint16_t i = 0;

        for (i = 0; i < dataLength; i++) {
            if (currentPacketLength == maxPacketSize) {
                break;
            }

            if (isSpecialCharacter(dataPtr[i])) {
                currentPacketLength += 2;
            } else {
                currentPacketLength++;
            }
        }

        bytesLeftToProcess = dataLength - i;
        return currentPacketLength;
    }
};
