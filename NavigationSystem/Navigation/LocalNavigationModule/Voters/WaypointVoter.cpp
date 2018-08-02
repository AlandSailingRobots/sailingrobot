/****************************************************************************************
 *
 * File:
 * 		WaypointVoter.cpp
 *
 * Purpose:
 *		
 *
 * License:
 *      This file is subject to the terms and conditions defined in the file 
 *      'LICENSE.txt', which is part of this source code package.
 *
 ***************************************************************************************/


#include "WaypointVoter.h"
#include "../ASRVoter.h"
#include "../Math/CourseMath.h"
#include "../SystemServices/Logger.h"

#include <iostream>


///----------------------------------------------------------------------------------
WaypointVoter::WaypointVoter( int16_t maxVotes, int16_t weight )
    :ASRVoter( maxVotes, weight, "Waypoint" )
{

}

///----------------------------------------------------------------------------------
const ASRCourseBallot& WaypointVoter::vote( const BoatState_t& boatState )
{
    double distance = CourseMath::calculateDTW( boatState.lon, boatState.lat, boatState.currWaypointLon, boatState.currWaypointLat );
    courseBallot.clear();

    Logger::debug("Bearing to WP: %d Distance to WP: %f", boatState.waypointBearing, distance);

    //std::cout << "Waypoint Votes: ";
    // Do i=0 first so its not doubled:
    int16_t votes = courseBallot.maxVotes();
    courseBallot.add( boatState.waypointBearing , votes );

    for( int i = 1; i < 90; i+= ASRCourseBallot::COURSE_RESOLUTION )
    {
        votes = courseBallot.maxVotes() - 0.4*( ( i / 90.f ) * (float)( courseBallot.maxVotes() ));

        //std::cout << "[" << waypointBearing + i << "]" << votes << ",";
        courseBallot.add( boatState.waypointBearing + i, votes );
        courseBallot.add( boatState.waypointBearing - i, votes );
    }

    return courseBallot;
}