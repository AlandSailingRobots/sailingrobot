/****************************************************************************************
 *
 * File:
 * 		VoterTCPDebugger.h
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


#ifndef NAVIGATIONSYSTEM_VOTERTCPDEBUGGER_H
#define NAVIGATIONSYSTEM_VOTERTCPDEBUGGER_H

#include "ASRVoter.h"
#include "../../Network/TCPServer.h"
#include "../../MessageBus/ActiveNode.h"
#include "../../SystemServices/Logger.h"
#include "../SystemServices/Timer.h"
#include "../LocalNavigationModule/LocalNavigationModule.h"
#include <string.h>


/*
struct VoterDataPacket_t {
    uint16_t courses[voter.courseBallot.ELEMENT_COUNT] = voter.courseBallot.courses;
    uint16_t veto[voter.courseBallot.ELEMENT_COUNT] = voter.courseBallot.veto;
    std::string voterName = voter.name;
    float voterWeight = voter.voterWeight;
} __attribute__((packed));
 */

struct VoterDataPacket_t {
    //int16_t courses[360]; //hardcoded for now, should be voter.courseBallot.ELEMENT_COUNT
    char c_courses[sizeof(int16_t[360])];
    bool veto[360];
    float voterWeight;
    char voterName[20];
} __attribute__((packed,aligned(2)));

class VoterTCPDebugger : public ActiveNode {
  public:
    VoterTCPDebugger(MessageBus& msgBus, ASRVoter& voter);
    VoterTCPDebugger(MessageBus& msgBus, LocalNavigationModule& lnm, int voter_pos_in_lnm_vector);

    void processMessage(const Message* msg);

    ///----------------------------------------------------------------------------------
    /// Initialize the TCP communication
    ///----------------------------------------------------------------------------------
    bool init();

    ///----------------------------------------------------------------------------------
    /// Starts the Node
    ///----------------------------------------------------------------------------------
    void start();

    ///----------------------------------------------------------------------------------
    /// Update voterDataPacket
    ///----------------------------------------------------------------------------------
    void updateMessage();

    ///----------------------------------------------------------------------------------
    /// Sends the voterDataPacket through the TCP connection
    ///----------------------------------------------------------------------------------
    void sendPacket(int socketFD);

    ///----------------------------------------------------------------------------------
    /// Main loop
    ///----------------------------------------------------------------------------------
    static void VoterTCPDebuggerThreadFunc(ActiveNode* nodePtr);


  private:
    ASRVoter* m_voter;
    TCPServer server;
    VoterDataPacket_t voterDataPacket;
    std::mutex m_lock;
};

#endif //NAVIGATIONSYSTEM_VOTERTCPDEBUGGER_H
