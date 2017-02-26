/****************************************************************************************
 *
 * File:
 * 		LocalNavigationModule.cpp
 *
 * Purpose:
 *		
 *
 * License:
 *      This file is subject to the terms and conditions defined in the file 
 *      'LICENSE.txt', which is part of this source code package.
 *
 ***************************************************************************************/


#include "LocalNavigationModule.h"
#include "Messages/DesiredCourseMsg.h"
#include "Messages/CompassDataMsg.h"
#include "Messages/GPSDataMsg.h"
#include "Messages/WindDataMsg.h"
#include "Messages/WaypointDataMsg.h"


///----------------------------------------------------------------------------------
LocalNavigationModule::LocalNavigationModule( MessageBus& msgBus )
    :Node(NodeID::LocalNavigationModule, msgBus)
{
    msgBus.registerNode( *this, MessageType::CompassData );
    msgBus.registerNode( *this, MessageType::GPSData );
    msgBus.registerNode( *this, MessageType::WindData );
    msgBus.registerNode( *this, MessageType::WaypointData );
    msgBus.registerNode( *this, MessageType::RequestCourse );
}

///----------------------------------------------------------------------------------
bool LocalNavigationModule::init() { return true; }

///----------------------------------------------------------------------------------
void LocalNavigationModule::processMessages( const Message* msg )
{
    switch( msg->messageType() )
    {
        case MessageType::CompassData:
            {
            CompassDataMsg* compass = (CompassDataMsg*)msg;
            boatState.heading = compass->heading();
            }
            break;

        case MessageType::GPSData:
            {
            GPSDataMsg* gps = (GPSDataMsg*)msg;
            boatState.lat = gps->latitude();
            boatState.lon = gps->longitude();
            boatState.speed = gps->speed();
            }
            break;

        case MessageType::WindData:
            {
            WindDataMsg* wind = (WindDataMsg*)msg;
            boatState.wind = wind->windDirection();
            }
            break;

        case MessageType::WaypointData:
            {
            WaypointDataMsg* waypoint = (WaypointDataMsg*)msg;
            boatState.currWaypointLat = waypoint->nextLatitude();
            boatState.currWaypointLon = waypoint->nextLongitude();

            boatState.lastWaypointLat = waypoint->prevLatitude();
            boatState.lastWaypointLon = waypoint->prevLongitude();

            boatState.radius = waypoint->nextRadius();
            // Delibrate dropdown, after a new waypoint we want to start a new ballot 
            // and get a new heading
            }
        case MessageType::RequestCourse:
            startBallot();
            break;
        default:
            break;
    }
}

///----------------------------------------------------------------------------------
void LocalNavigationModule::registerVoter( ASRVoter* voter )
{
    if( voter != NULL )
    {
        voters.push_back(voter);
    }
}

///----------------------------------------------------------------------------------
void LocalNavigationModule::startBallot()
{
    arbiter.clearBallot();

    std::vector<ASRVoter*>::iterator it;  // declare an iterator to a vector of strings

    for( it = voters.begin(); it != voters.end(); it++ ) 
    {
        arbiter.castVote( (*it)->vote( boatState ) );
    }

    DesiredCourseMsg msg( arbiter.getWinner() );
}
