/****************************************************************************************
 *
 * File:
 * 		WindVoter.cpp
 *
 * Purpose:
 *		
 *
 * License:
 *      This file is subject to the terms and conditions defined in the file 
 *      'LICENSE.txt', which is part of this source code package.
 *
 ***************************************************************************************/


#include "WindVoter.h"
#include "../SystemServices/Logger.h"
#include "../Math/Utility.h"


///----------------------------------------------------------------------------------
WindVoter::WindVoter( int16_t maxVotes, int16_t weight )
    :ASRVoter( maxVotes, weight, "Wind" )
{

}

///----------------------------------------------------------------------------------
const ASRCourseBallot& WindVoter::vote( const BoatState_t& boatState )
{
    courseBallot.clear();
    const int TACK_ANGLE = 45;
    uint16_t twd = Utility::getTrueWindDirection(boatState.windDir, boatState.windSpeed, 
                boatState.speed, boatState.heading, trueWindBuffer, TW_BUFFER_SIZE);

    /*
    // Set everything to 66% of the max vote
    for( int i = 0; i < 360; i+= ASRCourseBallot::COURSE_RESOLUTION )
    {
        courseBallot.set( i, courseBallot.maxVotes() / 1.5 );
    }
    */


    //int16_t twdBearingDiff = abs(Utility::headingDifference( boatState.waypointBearing, twd ));
    
    // Encourage tacking if necessary 

#ifdef on_ASPire  // defined in makefile, need to take into account downwind beating mode for ASPire

/*  Try using only the vetos
    if(abs(sin(Utility::degreeToRadian(twdBearingDiff))) <= abs(sin(Utility::degreeToRadian(TACK_ANGLE))))
    {
//        Logger::info("[Wind voter]: Beating mode needed");
        if(cos(Utility::degreeToRadian(twdBearingDiff)) >= 0) {
            if ( abs( Utility::headingDifference( boatState.heading, twd + TACK_ANGLE ) ) < 
                 abs( Utility::headingDifference( boatState.heading, twd - TACK_ANGLE ) ) )
            {
                courseBallot.add( twd + TACK_ANGLE, courseBallot.maxVotes() );
            }
            else
            {
                courseBallot.add( twd - TACK_ANGLE, courseBallot.maxVotes() );
            }
//            Logger::info("[Wind voter]: Beating mode downwind");
        } else {
            if ( abs( Utility::headingDifference( boatState.heading, twd + (180 - TACK_ANGLE) ) ) < 
                 abs( Utility::headingDifference( boatState.heading, twd - (180 - TACK_ANGLE) ) ) )
            {
                courseBallot.add( twd + (180 - TACK_ANGLE), courseBallot.maxVotes() );
            }
            else
            {
                courseBallot.add( twd - (180 - TACK_ANGLE), courseBallot.maxVotes() );
            }
//            Logger::info("[Wind voter]: Beating mode upwind");
        }
    }
*/

    // Set a veto to courses into the no go zones (upwind and downwind for wingsails).
    for( int i = 0; i < TACK_ANGLE; i+= ASRCourseBallot::COURSE_RESOLUTION )
    {
        courseBallot.setVeto( twd + i );
        courseBallot.setVeto( twd - i );
    }
    for( int i = 0; i < TACK_ANGLE; i+= ASRCourseBallot::COURSE_RESOLUTION )
    {
        courseBallot.setVeto( twd + 180 + i );
        courseBallot.setVeto( twd + 180 - i );
    }
#else         // for regular sails

/* Try using only the vetos
    if( twdBearingDiff <= TACK_ANGLE )
    {
        if ( abs( Utility::headingDifference( boatState.heading, twd + TACK_ANGLE ) ) < 
             abs( Utility::headingDifference( boatState.heading, twd - TACK_ANGLE ) ) )
        {
            courseBallot.add( twd + TACK_ANGLE, courseBallot.maxVotes() );
        }
        else
        {
            courseBallot.add( twd - TACK_ANGLE, courseBallot.maxVotes() );
        }
    }
*/
    // Set a veto to courses into the no go zone.
    for( int i = 0; i < TACK_ANGLE; i+= ASRCourseBallot::COURSE_RESOLUTION )
    {
        courseBallot.setVeto( twd + i );
        courseBallot.setVeto( twd - i );
    }

#endif //ASPire

    

    /*
    // Set 0 to courses into the no go zone.
    for( int i = 0; i < TACK_ANGLE; i+= ASRCourseBallot::COURSE_RESOLUTION )
    {
        courseBallot.set( twd + i, 0 );
        courseBallot.set( twd - i, 0 );
    }
    */

    

    return courseBallot;
}