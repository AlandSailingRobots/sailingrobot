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


#include "Navigation/LocalNavigationModule/ASRVoter.h"
#include "WorldState/CollidableMgr/CollidableMgr.h"


class ProximityVoterSuite; //forward declaration;

class ProximityVoter : public ASRVoter {
	friend ProximityVoterSuite;  // test class friend to allow for testing of private functions

public:
    ///----------------------------------------------------------------------------------
 	/// Constructs the proximity voter.
 	///----------------------------------------------------------------------------------
    ProximityVoter( int16_t maxVotes, int16_t weight, CollidableMgr& collidableMgr );

    ///----------------------------------------------------------------------------------
 	/// Triggers a ASR voter to place votes on the course headings.
 	///----------------------------------------------------------------------------------
    const ASRCourseBallot& vote( const BoatState_t& boatState );

private:
	void visualAvoidance();
    void avoidOutsideVisualField( int16_t visibleFieldLowBearingLimit, 
    	int16_t visibleFieldHighBearingLimit);
    void bearingAvoidanceSmoothed( int16_t bearing, uint16_t relativeObstacleDistance );
    void bearingPreferenceSmoothed( int16_t bearing, uint16_t relativeObstacleDistance );

    float aisAvoidance( const BoatState_t& boatState, AISCollidable_t& collidable );
   CollidableMgr& collidableMgr;
};