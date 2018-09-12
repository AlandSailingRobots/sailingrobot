/****************************************************************************************
 *
 * File:
 * 		ASRVoter.h
 *
 * Purpose:
 *		The abstract base voter class. A voter contains a course ballot and a vote
 *      function which needs to be overriden.
 *
 * License:
 *      This file is subject to the terms and conditions defined in the file
 *      'LICENSE.txt', which is part of this source code package.
 *
 ***************************************************************************************/

#pragma once

#include <string>
#include "ASRCourseBallot.h"
#include "BoatState.h"
#include <mutex>

class ASRVoter {
   public:
    friend class VoterTCPDebugger;
    friend class LocalNavigationModule;

    ASRVoter(int16_t maxVotes, float weight, const char* nameIn)
        : courseBallot(maxVotes), voterWeight(weight) {
        //name = nameIn;
        strncpy(name, nameIn, sizeof name - 1);
        name[19] = '\0';
    }

    ///----------------------------------------------------------------------------------
    /// Triggers a ASR voter to place votes on the course headings. This function returns
    /// a reference to the internal course ballot data.
    ///----------------------------------------------------------------------------------
    virtual const ASRCourseBallot& vote(const BoatState_t& boatState) = 0;

    ///----------------------------------------------------------------------------------
    /// Returns the voters weight, that was set during construction
    ///----------------------------------------------------------------------------------
    int16_t weight() { return voterWeight; }

    ///----------------------------------------------------------------------------------
    /// Returns the course with the highest number of votes. If there is a tie, the
    /// first course in the tie that is found is chosen.
    ///----------------------------------------------------------------------------------
    uint16_t getBestCourse(int16_t& value) {
        uint16_t bestCourse = 0;
        value = 0;
        for (uint16_t i = 0; i < 360; i++) {
            if (courseBallot.get(i) > value) {
                value = courseBallot.get(i);
                bestCourse = i;
            }
        }

        return bestCourse;
    }

    ///----------------------------------------------------------------------------------
    /// Returns the name of the voter
    ///----------------------------------------------------------------------------------
    const char* getName() { return name; }

    ///----------------------------------------------------------------------------------
    /// Returns the course ballot of the voter
    ///----------------------------------------------------------------------------------
    ASRCourseBallot* getBallot() { return &courseBallot; }


    //int access_tester;
    ASRCourseBallot courseBallot;
    float voterWeight;
    char name[20];
   protected:

    std::mutex m_lock;
};