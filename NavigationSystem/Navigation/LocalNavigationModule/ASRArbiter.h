/****************************************************************************************
 *
 * File:
 * 		ASRArbiter.h
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

#include <stdint.h>
#include "ASRCourseBallot.h"
#include <mutex>

class ASRArbiter {
   public:
    ///----------------------------------------------------------------------------------
    /// Constructs the Arbiter.
    ///----------------------------------------------------------------------------------
    ASRArbiter();

    ///----------------------------------------------------------------------------------
    /// Adds all the votes from a course ballot into its internal ballot. (and the vetos)
    ///----------------------------------------------------------------------------------
    void castVote(const int16_t weight, const ASRCourseBallot& ballot);

    ///----------------------------------------------------------------------------------
    /// Adds all the vetos from a course ballot into its internal ballot.
    /// NOTE: vetos added with cast vote, previous calls made the voters process twice
    ///----------------------------------------------------------------------------------
    void castVeto(const ASRCourseBallot& ballot);

    ///----------------------------------------------------------------------------------
    /// Returns the winning course.
    ///----------------------------------------------------------------------------------
    const uint16_t getWinner() const;

    ///----------------------------------------------------------------------------------
    /// Returns the summed results of all the voters that have cast their vote.
    ///----------------------------------------------------------------------------------
    const ASRCourseBallot& getResult() const;

    ///----------------------------------------------------------------------------------
    /// Clears the current ballot
    ///----------------------------------------------------------------------------------
    void clearBallot();

   private:
    const int MAX_VOTES = 150;
    ASRCourseBallot courseBallot;

    std::mutex m_lock;
};