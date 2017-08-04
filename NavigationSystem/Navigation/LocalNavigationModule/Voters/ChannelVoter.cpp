/****************************************************************************************
 *
 * File:
 * 		ChannelVoter.cpp
 *
 * Purpose:
 *
 *
 * License:
 *      This file is subject to the terms and conditions defined in the file
 *      'LICENSE.txt', which is part of this source code package.
 *
 ***************************************************************************************/


#include "ChannelVoter.h"


#include "Math/CourseMath.h"
#include "Math/Utility.h"
#include "SystemServices/Logger.h"
#include <cmath>

#include <iostream>


///----------------------------------------------------------------------------------
ChannelVoter::ChannelVoter( int16_t maxVotes, int16_t weight )
    :ASRVoter( maxVotes, weight, "Channel" )
{

}

///----------------------------------------------------------------------------------
const ASRCourseBallot& ChannelVoter::vote( const BoatState_t& boatState )
{
    //double distance = CourseMath::calculateDTW( boatState.lon, boatState.lat, boatState.currWaypointLon, boatState.currWaypointLat );
    courseBallot.clear();
    //const double CUTOFF = 0.8;
    static double maxDistanceFromLine = 0;

    double distanceFromMiddle = Utility::calculateSignedDistanceToLine( boatState.currWaypointLon,
                                boatState.currWaypointLat, boatState.lastWaypointLon,
                                boatState.lastWaypointLat, boatState.lon, boatState.lat );

    if(distanceFromMiddle > -3000)
    {
    if(maxDistanceFromLine < abs(distanceFromMiddle))
    {
        maxDistanceFromLine = abs(distanceFromMiddle);
    }
    }

    Logger::info("Max Distance From Line: %f Current distance from line: %f", maxDistanceFromLine, distanceFromMiddle);

    // double distanceRatio = distanceFromMiddle / boatState.radius;
    double waypointLineBearing = CourseMath::calculateBTW( boatState.lastWaypointLon, boatState.lastWaypointLat, boatState.currWaypointLon, boatState.currWaypointLat );

    // Logger::info("Last WP, lat: " + std::to_string(boatState.lastWaypointLat) + ", lon: " + std::to_string(boatState.lastWaypointLon));
    // Logger::info("WPBearing: " + std::to_string(waypointLineBearing) + ", Heading: " + std::to_string(boatState.heading) + ", Distance ratio: " + std::to_string(distanceRatio));

    // Left hand side
    if( distanceFromMiddle > boatState.radius - 3)
    {
        for( int i = 0; i < 45; i++ )
        {
            courseBallot.set( waypointLineBearing + 45 - i, courseBallot.maxVotes() );
            courseBallot.set( waypointLineBearing + 45 + i, courseBallot.maxVotes() );
        }
    }
    else if(distanceFromMiddle < -boatState.radius + 3 )
    {
        for( int i = 0; i < 45; i++ )
        {
            courseBallot.set( waypointLineBearing - 45 - i, courseBallot.maxVotes() );
            courseBallot.set( waypointLineBearing - 45 + i, courseBallot.maxVotes() );
        }
    }

    return courseBallot;
}
