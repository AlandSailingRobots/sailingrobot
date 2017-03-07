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


#include "utility/CourseMath.h"
#include "utility/Utility.h"
#include "SystemServices/Logger.h"

#include <iostream>


///----------------------------------------------------------------------------------
ChannelVoter::ChannelVoter( int16_t maxVotes, int16_t weight )
    :ASRVoter( maxVotes, weight )
{

}

///----------------------------------------------------------------------------------
const ASRCourseBallot& ChannelVoter::vote( const BoatState_t& boatState )
{
    //double distance = CourseMath::calculateDTW( boatState.lon, boatState.lat, boatState.currWaypointLon, boatState.currWaypointLat );
    courseBallot.clear();
    //const double CUTOFF = 0.8;

    double distanceFromMiddle = Utility::calculateSignedDistanceToLine( boatState.currWaypointLon, 
                                boatState.currWaypointLat, boatState.lastWaypointLon, 
                                boatState.lastWaypointLat, boatState.lon, boatState.lat );

    //double distanceRatio = distanceFromMiddle / boatState.radius;
    double waypointLineBearing = CourseMath::calculateBTW( boatState.lastWaypointLon, boatState.lastWaypointLat, boatState.currWaypointLon, boatState.currWaypointLat );

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

    /*for( int i = 0; i < 45; i++ )
    {
        int insideVote = (courseBallot.maxVotes() / 4);
        int outsideVote = 1 - ( ( ( i / 45 ) * ( i * i ) + (CUTOFF - distanceRatio ) ) * courseBallot.maxVotes() );

        // Left hand side of the line
        if(distanceRatio > 0 )
        {
            courseBallot.set( waypointLineBearing - 45 - i, outsideVote );
            courseBallot.set( waypointLineBearing - 45 + i, outsideVote );
            courseBallot.set( waypointLineBearing + 45 - i, insideVote );
            courseBallot.set( waypointLineBearing + 45 + i, insideVote );
        }
        else
        {
            courseBallot.set( waypointLineBearing + 45 - i, outsideVote );
            courseBallot.set( waypointLineBearing + 45 + i, outsideVote );
            courseBallot.set( waypointLineBearing - 45 - i, insideVote );
            courseBallot.set( waypointLineBearing - 45 + i, insideVote );
        }
    }*/

    return courseBallot;
}