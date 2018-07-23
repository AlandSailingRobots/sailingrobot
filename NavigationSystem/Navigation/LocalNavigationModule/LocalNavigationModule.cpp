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
#include "../Messages/LocalNavigationMsg.h"
#include "../Messages/StateMessage.h"
#include "../Messages/WindStateMsg.h"
#include "../Messages/WaypointDataMsg.h"
#include "../Messages/RequestCourseMsg.h"
#include "../SystemServices/Logger.h"
#include "../SystemServices/Timer.h"
#include "../Math/CourseMath.h"
#include "../Math/Utility.h"

#include <cstdio>


// For std::this_thread
#include <chrono>
#include <thread>

#define DATA_OUT_OF_RANGE -2000
#define WAKEUP_INTIAL_SLEEP     2000
const float NO_COMMAND = -1000;


///----------------------------------------------------------------------------------
LocalNavigationModule::LocalNavigationModule( MessageBus& msgBus,DBHandler& dbhandler)
    :ActiveNode(NodeID::LocalNavigationModule, msgBus), m_LoopTime(0.5), m_db(dbhandler), 
    m_trueWindDir(DATA_OUT_OF_RANGE)
{
    boatState.currWaypointLat = 0;
    boatState.currWaypointLon = 0;
    boatState.lastWaypointLat = 0;
    boatState.lastWaypointLon = 0;
    boatState.radius = 10;
    boatState.waypointBearing = 0;
    boatState.heading = 0;
    boatState.lat = 0;
    boatState.lon = 0;
    boatState.windDir = 0;
    boatState.windSpeed = 0;
    boatState.speed = 0;
    msgBus.registerNode( *this, MessageType::StateMessage );
    msgBus.registerNode( *this, MessageType::WindState );
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
    m_db.getConfigFrom(m_LoopTime, "loop_time", "config_voter_system");
}

///----------------------------------------------------------------------------------
void LocalNavigationModule::processMessage( const Message* msg )
{
    switch( msg->messageType() )
    {
        case MessageType::StateMessage:
        {
            StateMessage* vesselStateMsg = (StateMessage*)msg;
            boatState.heading = vesselStateMsg->heading();
            boatState.lat = vesselStateMsg->latitude();
            boatState.lon = vesselStateMsg->longitude();
            boatState.speed = vesselStateMsg->speed();
        }
            break;

        case MessageType::WindState:
        {
            WindStateMsg* windStateMsg = (WindStateMsg*)msg;
            boatState.windDir = windStateMsg->apparentWindDirection();
            boatState.windSpeed = windStateMsg->apparentWindSpeed();
            m_trueWindDir = windStateMsg->trueWindDirection();
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
            Logger::info( "Boat state: Lat: %f Lon: %f Heading: %f", boatState.lat, boatState.lon, boatState.heading );



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

//    std::cout << "Arbiter min/max: " << *std::min_element(std::begin(arbiter.getResult().courses),std::end(arbiter.getResult().courses)) << " "
//              <<  *std::max_element(std::begin(arbiter.getResult().courses),std::end(arbiter.getResult().courses)) << std::endl;

    std::vector<ASRVoter*>::iterator it;

    for( it = voters.begin(); it != voters.end(); it++ )
    {
        ASRVoter* voter = (*it);
        arbiter.castVote( voter->weight(), voter->vote( boatState ) );
        //arbiter.castVeto( voter->vote( boatState ) ); // vetos done with castVote now
        //voter->vote( boatState ).clear();
//        std::cout << "Arbiter min/max (after vote " << std::distance(std::begin(voters), it) << "): " << *std::min_element(std::begin(arbiter.getResult().courses),std::end(arbiter.getResult().courses)) << " "
//              <<  *std::max_element(std::begin(arbiter.getResult().courses),std::end(arbiter.getResult().courses)) << std::endl;
        // Debug/Tuning
        std::pair<int, int> minpair = voter->getBallot()->getMin();
        std::pair<int, int> maxpair = voter->getBallot()->getMax();
        std::cout << "Voter " << voter->getName().c_str() << " min: " << minpair.first << " " << minpair.second << " max: " << maxpair.first << " " << maxpair.second << std::endl;
    }
/*
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
*/


    uint16_t targetCourse = arbiter.getWinner();
    bool targetTackStarboard = getTargetTackStarboard((double) targetCourse);
    MessagePtr msg = std::make_unique<LocalNavigationMsg>((float) targetCourse, NO_COMMAND, 0, targetTackStarboard);
    m_MsgBus.sendMessage( std::move( msg ) );
}

///----------------------------------------------------------------------------------
bool LocalNavigationModule::getTargetTackStarboard(double targetCourse)
{
    if (sin(Utility::degreeToRadian(targetCourse - m_trueWindDir)) < 0){
        return true;
    } else {
        return false;
    }
}

///----------------------------------------------------------------------------------
/// Just a little hack for waking up the navigation module for now
///----------------------------------------------------------------------------------
void LocalNavigationModule::WakeupThreadFunc( ActiveNode* nodePtr )
{
    LocalNavigationModule* node = dynamic_cast<LocalNavigationModule*> (nodePtr);

	// An initial sleep, its purpose is to ensure that most if not all the sensor data arrives
	// at the start before we send out the vessel state message.
	std::this_thread::sleep_for( std::chrono::milliseconds( WAKEUP_INTIAL_SLEEP ) );

    Timer timer;
    timer.start();

	while(true)
	{
        MessagePtr courseRequest = std::make_unique<RequestCourseMsg>();
		node->m_MsgBus.sendMessage( std::move( courseRequest ) );

        // Controls how often we pump out messages
        timer.sleepUntil(node->m_LoopTime);
        timer.reset();
	}
}

