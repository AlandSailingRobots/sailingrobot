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
#include "Math/CourseMath.h"

#include <cstdio>


// For std::this_thread
#include <chrono>
#include <thread>


#define WAKEUP_SLEEP_MS         400
#define WAKEUP_INTIAL_SLEEP     2000


///----------------------------------------------------------------------------------
LocalNavigationModule::LocalNavigationModule( MessageBus& msgBus,DBHandler& dbhandler)
    :ActiveNode(NodeID::LocalNavigationModule, msgBus), m_LoopTime(0.5), m_db(dbhandler)
{
    msgBus.registerNode( *this, MessageType::CompassData );
    msgBus.registerNode( *this, MessageType::GPSData );
    msgBus.registerNode( *this, MessageType::WindData );
    msgBus.registerNode( *this, MessageType::WaypointData );
    msgBus.registerNode( *this, MessageType::RequestCourse );
    msgBus.registerNode( *this, MessageType::ServerConfigsReceived );
}

///----------------------------------------------------------------------------------
bool LocalNavigationModule::init()
{
    updateConfigsFromDB();
    return true;
}

///----------------------------------------------------------------------------------
void LocalNavigationModule::start()
{
    runThread(WakeupThreadFunc);
}

///----------------------------------------------------------------------------------
void LocalNavigationModule::updateConfigsFromDB(){
    m_LoopTime = m_db.retrieveCellAsDouble("config_voter_system","1","loop_time");
}

///----------------------------------------------------------------------------------
void LocalNavigationModule::processMessage( const Message* msg )
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

            // Delibrate dropdown after a new waypoint, we want to start a new ballot
            // and get a new heading
        }
        case MessageType::RequestCourse:
            startBallot();
            break;
        case MessageType::ServerConfigsReceived:
            updateConfigsFromDB();
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

    std::vector<ASRVoter*>::iterator it;

    for( it = voters.begin(); it != voters.end(); it++ )
    {
        ASRVoter* voter = (*it);
        arbiter.castVote( voter->weight(), voter->vote( boatState ) );
    }

    printf("[Voters] "); // Debug
    for( it = voters.begin(); it != voters.end(); it++ )
    {
        ASRVoter* voter = (*it);
        int16_t votes = 0;
        uint16_t bestCourse = voter->getBestCourse(votes);

        std::string name = voter->getName();
        printf("%s : %d %d ", name.c_str(), bestCourse, votes); // Debug: Prints out some voting information
    }
    printf("\n");

    MessagePtr msg = std::make_unique<DesiredCourseMsg>( arbiter.getWinner() );
    m_MsgBus.sendMessage( std::move( msg ) );
}

///----------------------------------------------------------------------------------
/// Just a little hack for waking up the navigation module for now
///----------------------------------------------------------------------------------
void LocalNavigationModule::WakeupThreadFunc( ActiveNode* nodePtr )
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
