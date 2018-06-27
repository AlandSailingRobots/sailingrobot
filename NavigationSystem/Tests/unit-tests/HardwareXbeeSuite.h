/****************************************************************************************
 *
 * File:
 * 		HardwareXbeeSuite.h
 *
 * Purpose:
 *		A set of unit tests for checking whether the GPS Node is functioning correctly
 *		These set of tests will only work if the GPS is physically attached
 *
 * Developer Notes:
 *
 *							12.4.17 JM
 *
 *	Functions that have tests:		Functions that does not have tests:
 *
 *	init 							writeData
 *	setIncomingCallback 			readData
 *	processRadioMessages 			dataAvailable
 *	transmit 						slip
 *	receivePackets 					deslip
 *	processPacketQueue
 *	processPacket
 *	fletcherChecksum
 *
 ***************************************************************************************/

#pragma once

#include <stdint.h>
#include <thread>
#include "../SystemServices/Logger.h"
#include "../Xbee/Xbee.h"
#include "../cxxtest/cxxtest/TestSuite.h"

// For std::this_thread
#include <chrono>
#include <thread>

#define XBEE_PORT "/dev/ttyUSB0"

class MockXbee : public Xbee {
   public:
    MockXbee() : Xbee(true) {}
    virtual ~MockXbee() {}

    std::vector<XbeePacket> packets;
    unsigned int index = 0;

    void w_processPackets(XbeePacket& packet) { processPacket(packet); }
    void w_processMultiPacket(std::vector<XbeePacket>& packets) { processPacket(packets); }

    void w_processPacketQueue() { processPacketQueue(); }
    void w_receivePackets() { receivePackets(); }

    uint16_t queueSize() { return m_receiveQueue.size(); }

    uint16_t w_fletcherChecksum(uint8_t* data, uint16_t size) {
        return fletcherChecksum(data, size);
    }

   protected:
    virtual void writeData(XbeePacket packet) {
        uint8_t* data = new uint8_t[packet.m_payloadSize];
        memcpy(data, packet.m_payload, packet.m_payloadSize);
        packet.m_payload = data;

        packets.push_back(packet);
    }

    virtual uint8_t readData(XbeePacket& packet) {
        if (index < packets.size()) {
            packet = packets[index++];
            return packet.m_payloadSize;
        }

        return 0;
    }
};

class HardwareXbeeSuite : public CxxTest::TestSuite {
   public:
    MockXbee* xbee;

    static uint8_t* lastReceived(uint8_t* l = 0) {
        static uint8_t* lr = 0;

        if (l != 0) {
            lr = l;
        }

        return lr;
    }

    static uint8_t lastSize(uint8_t l = 0) {
        static uint8_t lr = 0;

        if (l != 0) {
            lr = l;
        }

        return lr;
    }

    static void inboundMessage(uint8_t* data, uint8_t size) {
        lastReceived(data);
        lastSize(size);
    }

    void setUp() {
        lastReceived();
        lastSize();
        xbee = new MockXbee();
        xbee->setIncomingCallback(inboundMessage);
        Logger::DisableLogging();
    }

    void tearDown() { delete xbee; }

    void test_BrexitVotePassed() { TS_ASSERT(true); }

    void test_EndOfUK() { TS_ASSERT(true); }

    void test_Init() { TS_ASSERT(xbee->init(XBEE_PORT, 57600)); }

    void test_SinglePacketData() {
        uint8_t data[] = {1, 1, 1, 1, 1, 1, 1, 1};

        xbee->transmit(data, sizeof(data));
        xbee->processRadioMessages();

        TS_ASSERT_EQUALS(xbee->packets.size(), 1);
    }

    void test_MultiPacketData() {
        uint8_t data[73];

        xbee->transmit(data, 73);
        xbee->processRadioMessages();

        TS_ASSERT_EQUALS(xbee->packets.size(), 2);
        TS_ASSERT_EQUALS(xbee->packets[0].m_packetCount, 2);
        TS_ASSERT_EQUALS(xbee->packets[1].m_packetCount, 2);
        TS_ASSERT_EQUALS(xbee->packets[0].m_packetID, xbee->packets[1].m_packetID);
    }

    void test_PacketCountIncrement() {
        uint8_t data[] = {1, 1, 1, 1, 1, 1, 1, 1};

        xbee->transmit(data, sizeof(data));
        xbee->processRadioMessages();

        TS_ASSERT_EQUALS(xbee->packets[0].m_packetID, 0);

        xbee->transmit(data, sizeof(data));
        xbee->processRadioMessages();

        TS_ASSERT_EQUALS(xbee->packets[1].m_packetID, 1);

        xbee->transmit(data, sizeof(data));
        xbee->processRadioMessages();

        TS_ASSERT_EQUALS(xbee->packets[2].m_packetID, 2);
    }

    void test_PacketReceive() {
        uint8_t data[] = {1, 1, 1, 1, 1, 1, 1, 1};
        xbee->transmit(data, sizeof(data));
        xbee->processRadioMessages();
        xbee->w_receivePackets();
        TS_ASSERT_EQUALS(xbee->queueSize(), 1);
    }

    void test_MultiPacketReceive() {
        uint8_t data[73];
        xbee->transmit(data, 73);
        xbee->processRadioMessages();

        xbee->w_receivePackets();
        TS_ASSERT_EQUALS(xbee->queueSize(), 2);
    }

    void test_CallbackSinglePacket() {
        uint8_t data[] = {1, 1, 3, 1, 1, 1, 1, 1};
        xbee->transmit(data, sizeof(data));
        xbee->processRadioMessages();

        xbee->w_receivePackets();
        TS_ASSERT_EQUALS(xbee->queueSize(), 1);
        xbee->w_processPacketQueue();

        TS_ASSERT_EQUALS(lastSize(), sizeof(data));
        TS_ASSERT_EQUALS(lastReceived()[2], data[2]);
    }

    void test_CallbackMultiPacket() {
#define PACKET_SIZE 100
        uint8_t data[PACKET_SIZE];
        memset(data, 0, PACKET_SIZE);
        data[70] = 9;
        xbee->transmit(data, PACKET_SIZE);
        xbee->processRadioMessages();

        xbee->w_receivePackets();
        TS_ASSERT_EQUALS(xbee->queueSize(), 2);
        xbee->w_processPacketQueue();
        TS_ASSERT_EQUALS(lastSize(), PACKET_SIZE);
        TS_ASSERT_EQUALS(lastReceived()[70], 9);
    }

    void test_FletcherChecksum() {
        uint8_t data[] = {0x01, 0x02};

        uint16_t checksum = xbee->w_fletcherChecksum(data, sizeof(data));

        TS_ASSERT_EQUALS(checksum, 0x0403);

        char str[] = "Oh Borris, why did you make us leave the EU, WHY!!";

        checksum = xbee->w_fletcherChecksum((uint8_t*)str, sizeof(str));
        TS_ASSERT_EQUALS(checksum, 33630);
    }
};
