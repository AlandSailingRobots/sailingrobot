/****************************************************************************************
 *
 * File:
 * 		ProximityVoter.h
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
#include "CollidableMgr/CollidableMgr.h"


class ProximityVoter : public ASRVoter {
public:
    ///----------------------------------------------------------------------------------
 	/// Constructs the proximity voter.
 	///----------------------------------------------------------------------------------
    ProximityVoter( int16_t maxVotes, int16_t weight, CollidableMgr& collidableMgr );

    ///----------------------------------------------------------------------------------
 	/// Triggers a ASR voter to place votes on the course headings.
 	///----------------------------------------------------------------------------------
    const ASRCourseBallot& vote( const BoatState_t& boatState );

    void bearingAvoidance( const BoatState_t& boatState, uint16_t bearing );

    float aisAvoidance( const BoatState_t& boatState, AISCollidable_t& collidable );
private:
    CollidableMgr& collidableMgr;
};