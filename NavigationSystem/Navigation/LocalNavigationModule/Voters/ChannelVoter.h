/****************************************************************************************
 *
 * File:
 * 		ChannelVoter.h
 *
 * Purpose:
 *
 *
 * License:
 *      This file is subject to the terms and conditions defined in the file
 *      'LICENSE.txt', which is part of this source code package.
 *
 ***************************************************************************************/

#pragma once

#include "Navigation/LocalNavigationModule/ASRVoter.h"

class ChannelVoter : public ASRVoter {
   public:
    ///----------------------------------------------------------------------------------
    /// Constructs the channel voter.
    ///----------------------------------------------------------------------------------
    ChannelVoter(int16_t maxVotes, int16_t weight);

    ///----------------------------------------------------------------------------------
    /// Triggers a ASR voter to place votes on the course headings. The wind voter
    /// places voters on courses that allow the boat to go faster
    ///----------------------------------------------------------------------------------
    const ASRCourseBallot& vote(const BoatState_t& boatState);
};