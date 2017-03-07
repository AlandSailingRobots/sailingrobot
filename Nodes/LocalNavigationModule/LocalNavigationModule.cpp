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
#include "Messages/RequestCourseMsg.h"
#include "SystemServices/Logger.h"
#include "utility/CourseMath.h"

#include <cstdio>


// For std::this_thread
#include <chrono>
#include <thread>


#define WAKEUP_SLEEP_MS         1000
#define WAKEUP_INTIAL_SLEEP     2000


FILE* file = fopen("./gps.txt", "w");


///----------------------------------------------------------------------------------
LocalNavigationModule::LocalNavigationModule( MessageBus& msgBus )
    :ActiveNode(NodeID::LocalNavigationModule, msgBus)
{
    msgBus.registerNode( *this, MessageType::CompassData );
    msgBus.registerNode( *this, MessageType::GPSData );
    msgBus.registerNode( *this, MessageType::WindData );
    msgBus.registerNode( *this, MessageType::WaypointData );
    msgBus.registerNode( *this, MessageType::RequestCourse );

    fprintf( file, "%s,%ss,%s\n", "id", "latitude", "longitude" );
    fflush( file );
}

///----------------------------------------------------------------------------------
bool LocalNavigationModule::init() { return true; }

///----------------------------------------------------------------------------------
void LocalNavigationModule::start()
{
    runThread(WakeupThreadFunc);
}

///----------------------------------------------------------------------------------
void LocalNavigationModule::processMessage( const Message* msg )
{
    static int i = 0;
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

            fprintf( file, "%d,%f,%f\n", i, boatState.lat, boatState.lon );
            fflush( file );
        }
            break;

        case MessageType::WindData:
        {
            WindDataMsg* wind = (WindDataMsg*)msg;
            boatState.windDir = wind->windDirection();
            boatState.windSpeed = wind->windSpeed();
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
            double distance = CourseMath::calculateDTW( boatState.lon, boatState.lat, boatState.currWaypointLon, boatState.currWaypointLat );
            Logger::info( "New Waypoint! Lat: %f Lon: %f Distance: %f", boatState.currWaypointLat, boatState.currWaypointLon, distance );

            // Delibrate dropdown after a new waypoint,0 we want to start a new ballot 
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
    boatState.waypointBearing = CourseMath::calculateBTW( boatState.lon, boatState.lat, boatState.currWaypointLon, boatState.currWaypointLat );

    std::vector<ASRVoter*>::iterator it;  // declare an iterator to a vector of strings

    for( it = voters.begin(); it != voters.end(); it++ ) 
    {
        ASRVoter* voter = (*it);

        arbiter.castVote( voter->weight(), voter->vote( boatState ) );
    }

    MessagePtr msg = std::make_unique<DesiredCourseMsg>( arbiter.getWinner() );
    m_MsgBus.sendMessage( std::move( msg ) );
}

///----------------------------------------------------------------------------------
/// Just a little hack for waking up the navigation module for now
///----------------------------------------------------------------------------------
void LocalNavigationModule::WakeupThreadFunc( void* nodePtr )
{
    LocalNavigationModule* node = (LocalNavigationModule*)nodePtr;

	// An initial sleep, its purpose is to ensure that most if not all the sensor data arrives
	// at the start before we send out the vessel state message.
	std::this_thread::sleep_for( std::chrono::milliseconds( WAKEUP_INTIAL_SLEEP ) );

	while(true)
	{
		// Controls how often we pump out messages
		std::this_thread::sleep_for( std::chrono::milliseconds( WAKEUP_SLEEP_MS ) );

        MessagePtr courseRequest = std::make_unique<RequestCourseMsg>();
		node->m_MsgBus.sendMessage( std::move( courseRequest ) );
	}
}