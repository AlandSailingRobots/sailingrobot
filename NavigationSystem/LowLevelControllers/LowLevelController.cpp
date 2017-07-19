/****************************************************************************************
 *
 * File:
 * 		LowLevelController.cpp
 *
 * Purpose:
 *
 *
 * License:
 *      This file is subject to the terms and conditions defined in the file
 *      'LICENSE.txt', which is part of this source code package.
 *
 ***************************************************************************************/


#include "LowLevelController.h"

#include <stdlib.h>
#include "Math/Utility.h"
#include "Messages/CompassDataMsg.h"
#include "Messages/WindDataMsg.h"
#include "Messages/DesiredCourseMsg.h"
#include "Messages/ActuatorPositionMsg.h"
#include "SystemServices/Logger.h"


#define NORMALISE_RUDDER( val ) (float)val / (float)MAX_RUDDER_ANGLE


///----------------------------------------------------------------------------------
LowLevelController::LowLevelController( MessageBus& msgBus, DBHandler& dbHandler, double configPGain, double configIGain )
    :Node(NodeID::LowLevelController, msgBus), heading(HEADING_ERROR_VALUE),
    desiredHeading(HEADING_ERROR_VALUE), pGain(configPGain), iGain(configIGain)
{
    msgBus.registerNode( *this, MessageType::CompassData );
    msgBus.registerNode( *this, MessageType::WindData );
    msgBus.registerNode( *this, MessageType::DesiredCourse );

    //closeRange_ms = dbHandler.retrieveCellAsInt( "sail_command_config", "1", "close_reach_command" );
    //sailRange_ms = dbHandler.retrieveCellAsInt("sail_command_config", "1", "run_command") - closeRange_ms;
    //rudderMidpoint_ms = dbHandler.retrieveCellAsInt("rudder_command_config", "1", "midship_command"); // Not use in DataBase
    // rudderRange_ms = dbHandler.retrieveCellAsInt("rudder_command_config", "1","extreme_command") - rudderMidpoint_ms;
}

///----------------------------------------------------------------------------------
void LowLevelController::processMessage( const Message* msg )
{
    switch( msg->messageType() )
    {
        case MessageType::CompassData:
        {
            CompassDataMsg* compass = (CompassDataMsg*)msg;
            heading = compass->heading();
            calculateRudder();
            sendActuatorMsg();
            //Logger::info("Desired Course: %d Heading: %d", desiredHeading, heading);
        }
            break;
        case MessageType::WindData:
        {
            WindDataMsg* wind = (WindDataMsg*)msg;
            calculateSail( wind->windDirection() );
            sendActuatorMsg();
        }
            break;
        case MessageType::DesiredCourse:
        {
            DesiredCourseMsg* courseMsg = (DesiredCourseMsg*)msg;
            desiredHeading = courseMsg->desiredCourse();
        }
            break;
        default:
            break;
    }
}

///----------------------------------------------------------------------------------
void LowLevelController::sendActuatorMsg()
{
    MessagePtr actuatorMsg = std::make_unique<ActuatorPositionMsg>( rudder_ms, sail_ms );
    m_MsgBus.sendMessage( std::move( actuatorMsg ) );
}

///----------------------------------------------------------------------------------
void LowLevelController::calculateRudder()
{
    // We convert the PI controller value into a normalised value which we then
    // apply to the range the rudder can move. This range is then added to the
    // midpoint value giving us our final rudder position in maestro's pwm range.
    rudder_ms = rudderMidpoint_ms + ( NORMALISE_RUDDER( pi() ) * rudderRange_ms );
}

///----------------------------------------------------------------------------------
void LowLevelController::calculateSail( int windDir )
{
    // Fixed sail angles normalised
    static const int closeHauled = 0;
    static const int closeReach  = 20;
    static const int beamReach   = 50;
    static const int broadReach  = 70;
    static const int running     = 100;

    int command = 0;

    if( windDir < 180)
    {
        if( windDir < 45 ) { command = closeHauled; } // Logger::info("Sail is close hauled"); }
        else if( windDir < 70 ) { command = closeReach; } //Logger::info("Sail is close reached"); }
        else if( windDir < 95 ) { command = beamReach; } //Logger::info("Sail is beam reach"); }
        else if( windDir < 110 ) { command = broadReach; } // Logger::info("Sail is broad reach"); }
        else  { command = running; } //Logger::info("Sail is running"); }
        //if( windDir < 135 )
    }
    else
    {
        if( windDir < 225 ) { command = running; } // Logger::info("Sail is running"); }
        else if( windDir < 250 ) { command = broadReach; } // Logger::info("Sail is broad reach"); }
        else if( windDir < 265 ) { command = beamReach;}// Logger::info("Sail is beam reach"); }
        else if( windDir < 290 ) { command = closeReach;}// Logger::info("Sail is close reached"); }
        else { command = closeHauled;}// Logger::info("Sail is close hauled"); }
        //if( windDir < 335 )
    }

    // the / 100 puts us into the correct range, and prevents floating point maths
    sail_ms = closeRange_ms + ( ( command * sailRange_ms ) / 100);
}

///----------------------------------------------------------------------------------
int16_t LowLevelController::restrictRudder( int16_t val )
{
    if( val > MAX_RUDDER_ANGLE)
    {
        return MAX_RUDDER_ANGLE;
    }
    else if ( val < -MAX_RUDDER_ANGLE)
    {
        return -MAX_RUDDER_ANGLE;
    }

    return val;
}


///----------------------------------------------------------------------------------
int16_t LowLevelController::pi()
{
    static int16_t integral = 0;
    const int16_t MAX_INTEGRAL = 10;
    int16_t error = 0;

    if( heading == HEADING_ERROR_VALUE ) { return 0; }
    if( desiredHeading == HEADING_ERROR_VALUE) { return heading; }

    error = Utility::headingDifference( heading, desiredHeading );

    // TODO : value of 0.25 ???
    integral = integral + ( error * 0.25 );

    if( integral < -MAX_INTEGRAL )
    {
        integral = -MAX_INTEGRAL;
    }
    else if(integral > MAX_INTEGRAL )
    {
        integral = MAX_INTEGRAL;
    }

    int16_t p = error * pGain;

    int16_t i = integral * iGain;

    //Logger::info("Desired Course: %d Heading: %d Rudder Angle: %d PI Integral: %d", desiredHeading, heading, restrictRudder(p + i), integral);

    // Restrict to the angles the rudder can actually move to
    return restrictRudder(p + i);
}
