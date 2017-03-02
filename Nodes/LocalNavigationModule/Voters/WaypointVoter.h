/****************************************************************************************
 *
 * File:
 * 		WaypointVoter.h
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


#include "../ASRVoter.h"
#include "utility/CourseMath.h"


class WaypointVoter : public ASRVoter {
public:
    ///----------------------------------------------------------------------------------
 	/// Constructs the waypoint voter.
 	///----------------------------------------------------------------------------------
    WaypointVoter( int16_t maxVotes, int16_t weight );

    ///----------------------------------------------------------------------------------
 	/// Triggers a ASR voter to place votes on the course headings. The waypoint voter
    /// places voters on courses that bring the boat towards the waypoint.
 	///----------------------------------------------------------------------------------
    const ASRCourseBallot& vote( BoatState_t& boatState );
private:
};