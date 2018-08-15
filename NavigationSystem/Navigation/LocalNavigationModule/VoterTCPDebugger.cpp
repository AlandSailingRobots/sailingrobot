/****************************************************************************************
 *
 * File:
 * 		VoterTCPDebugger.cpp
 *
 * Purpose:
 *		Send voters data to a python script using TCP connection. The python script
 *		is used for debugging purposes like plotting data.
 *
 * License:
 *      This file is subject to the terms and conditions defined in the file
 *      'LICENSE.txt', which is part of this source code package.
 *
 ***************************************************************************************/

#include "VoterTCPDebugger.h"

#define SERVER_PORT 58888

VoterTCPDebugger::VoterTCPDebugger(MessageBus& msgBus, ASRVoter& voter)
: ActiveNode(NodeID::VoterTCPDebugger, msgBus),
  m_voter(&voter) {}


VoterTCPDebugger::VoterTCPDebugger(MessageBus& msgBus, LocalNavigationModule& lnm, int voter_pos_in_lnm_vector)
        : ActiveNode(NodeID::VoterTCPDebugger, msgBus),
          m_voter(NULL)
           {

    std::vector<ASRVoter*>::iterator it;
    it = lnm.voters.begin() + voter_pos_in_lnm_vector;
    m_voter = (*it);

}

void VoterTCPDebugger::start() {
    runThread(VoterTCPDebuggerThreadFunc);
}

bool VoterTCPDebugger::init() {
    bool success = false;

    int rc = server.start(SERVER_PORT);

    if (rc > 0) {
        Logger::info("Waiting for debugger client...\n");

        // Block until connection, don't timeout
        server.acceptConnection(0);

        success = true;
    } else {
        Logger::error("Failed to start the voter debug server");
        success = false;
    }

    return success;
}

///----------------------------------------------------------------------------------
void VoterTCPDebugger::updateMessage() {
    std::lock_guard<std::mutex> lock_guard(m_lock);

    int16_t (*ptr)[360] = &(m_voter->getBallot()->courses);
    char c_courses[720];
    char (*c_ptr)[720] = &c_courses;
    c_ptr = reinterpret_cast<char(*)[720]>(ptr);
    std::copy(std::begin(*c_ptr), std::end(*c_ptr), std::begin(voterDataPacket.c_courses));
    std::copy(std::begin(m_voter->getBallot()->veto), std::end(m_voter->getBallot()->veto), std::begin(voterDataPacket.veto));
    voterDataPacket.voterWeight = m_voter->weight();
    strcpy(voterDataPacket.voterName,m_voter->getName());


    /*
     * Working test code for debugging
     */
    /*
    int16_t ones[360];
    std::fill_n(ones, 360, 11);
    int16_t (*ptr)[360] = &ones;
    char c_ones[720];
    char (*c_ptr)[720] = &c_ones;
    c_ptr = reinterpret_cast<char(*)[720]>(ptr);
    bool bools[360];
    std::fill_n(bools, 360, true);
    float weight = 3.0;
    char name[20] = "hey_there";
    std::copy(std::begin(*c_ptr), std::end(*c_ptr), std::begin(voterDataPacket.c_courses));
    std::copy(std::begin(bools), std::end(bools), std::begin(voterDataPacket.veto));
    voterDataPacket.voterWeight = weight;
    strcpy(voterDataPacket.voterName,name);
    */

}

///--------------------------------------------------------------------------------------
void VoterTCPDebugger::sendPacket(int socketFD) {
    std::cout << "Size of packet: " << sizeof(VoterDataPacket_t) << std::endl;
    server.sendData(socketFD, &voterDataPacket, sizeof(VoterDataPacket_t));
}

///--------------------------------------------------------------------------------------
void VoterTCPDebugger::VoterTCPDebuggerThreadFunc(ActiveNode* nodePtr) {
    VoterTCPDebugger* node = dynamic_cast<VoterTCPDebugger*>(nodePtr);

    TCPPacket_t packet;
    int voterTCPDebuggerFD = 0;

    Timer timer;
    timer.start();

    while (true) {
        // Don't timeout on a packet read
        std::cout << "Hello" << std::endl;
        node->server.readPacket(packet, 0);

        // We only care about the latest packet, so clear out the old ones
        node->server.clearSocketBuffer( packet.socketFD );

        // We can safely assume that the first packet we receive will actually be from
        // the simulator as we only should ever accept one connection, the first one/
        if (voterTCPDebuggerFD == 0) {
            voterTCPDebuggerFD = packet.socketFD;
        }
        // Reset our packet, better safe than sorry
        packet.socketFD = 0;
        packet.length = 0;

        node->updateMessage();

        node->sendPacket(voterTCPDebuggerFD);

        timer.sleepUntil(0.1);
        timer.reset();
    }
}

///--------------------------------------------------------------------------------------
void VoterTCPDebugger::processMessage(const Message *msg) { return; }
