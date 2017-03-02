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
#include "SystemServices/Logger.h"
#include "utility/Utility.h"


///----------------------------------------------------------------------------------
WindVoter::WindVoter( int16_t maxVotes, int16_t weight )
    :ASRVoter( maxVotes, weight )
{

}

///----------------------------------------------------------------------------------
const ASRCourseBallot& WindVoter::vote( BoatState_t& boatState )
{
    const int TACK_ANGLE = 45;
    uint16_t twd = Utility::getTrueWindDirection(boatState.windDir, boatState.windSpeed, 
                boatState.speed, boatState.heading, trueWindBuffer, TW_BUFFER_SIZE);

    Logger::info("True Wind: %d Wind dir: %d", twd, boatState.windDir );

    // Set everything to max votes
    for( int i = 0; i < 360; i+= ASRCourseBallot::COURSE_RESOLUTION )
    {
        courseBallot.set( i, courseBallot.maxVotes() / 1.5 );
    }
 
    /*for ( int i = 0; i < 6; i+= ASRCourseBallot::COURSE_RESOLUTION )
    {
        //courseBallot.set( twd - 90 + i, courseBallot.maxVotes() / 4 );
        courseBallot.add( twd - 90 - i, courseBallot.maxVotes() / 4 );
        courseBallot.add( twd + 90 + i, courseBallot.maxVotes() / 4 );
        //courseBallot.set( twd + 90 - i, courseBallot.maxVotes() / 4 );
    }*/

    // Set 0 to courses into the no go zone.
    for( int i = 0; i < TACK_ANGLE; i+= ASRCourseBallot::COURSE_RESOLUTION )
    {
        courseBallot.set( twd + i, 0 );
        courseBallot.set( twd - i, 0 );
    }

    return courseBallot;
}