/****************************************************************************************
 *
 * File:
 * 		MidRangeVoter.h
 *
 * Note:
 *      Based around Jose C. Alves and Nuno A. Cruz's work in the IRSC 2015 paper
 *      "AIS=Enabled Collision Avoidance Strategies for Autonomous Sailboats"
 *
 * License:
 *      This file is subject to the terms and conditions defined in the file
 *      'LICENSE.txt', which is part of this source code package.
 *
 ***************************************************************************************/

#pragma once

#include "Navigation/LocalNavigationModule/ASRVoter.h"
#include "WorldState/CollidableMgr/CollidableMgr.h"

class MidRangeVoter : public ASRVoter {
   public:
    ///----------------------------------------------------------------------------------
    /// Constructs the Mid-Range voter.
    ///----------------------------------------------------------------------------------
    MidRangeVoter(int16_t maxVotes, int16_t weight, CollidableMgr& collisionMgr);

    ///----------------------------------------------------------------------------------
    /// Triggers a ASR voter to place votes on the course headings.
    ///----------------------------------------------------------------------------------
    const ASRCourseBallot& vote(const BoatState_t& boatState);

    const void assignVotes(uint16_t course, float collisionRisk);

    ///----------------------------------------------------------------------------------
    /// Finds the closest point of approach, the final parameter is the time until
    /// approach.
    ///----------------------------------------------------------------------------------
    const double getCPA(const AISCollidable_t& collidable,
                        const BoatState_t& boatState,
                        uint16_t course,
                        double& time);

   private:
    CollidableMgr& collidableMgr;
};