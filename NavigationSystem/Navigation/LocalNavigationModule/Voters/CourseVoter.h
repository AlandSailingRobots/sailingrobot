/****************************************************************************************
 *
 * File:
 * 		CourseVoter.h
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
#include "Math/CourseMath.h"

class CourseVoter : public ASRVoter {
   public:
    ///----------------------------------------------------------------------------------
    /// Constructs the course voter.
    ///----------------------------------------------------------------------------------
    CourseVoter(int16_t maxVotes, int16_t weight);

    ///----------------------------------------------------------------------------------
    /// Triggers a ASR voter to place votes on the course headings. The course voter
    /// places votes to the direction the boat is facing, less cost to change the vessel.
    ///----------------------------------------------------------------------------------
    const ASRCourseBallot& vote(const BoatState_t& boatState);

   private:
}; 
