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
#include "utility/Utility.h"
#include "Messages/CompassDataMsg.h"
#include "Messages/DesiredCourseMsg.h"


///----------------------------------------------------------------------------------
LowLevelController::LowLevelController( MessageBus& msgBus, double configPGain, double configIGain )
    :Node(NodeID::LowLevelController, msgBus), heading(370), desiredHeading(370), pGain(configPGain), 
    iGain(configIGain)
{
    msgBus.registerNode( *this, MessageType::CompassData );
    msgBus.registerNode( *this, MessageType::DesiredCourse );
}

///----------------------------------------------------------------------------------
void LowLevelController::processMessage( const Message* msg )
{
    uint16_t command = 370;

    switch( msg->messageType() )
    {
        case MessageType::CompassData:
        {
            CompassDataMsg* compass = (CompassDataMsg*)msg;
            heading = compass->heading();
            command = pi();
        }
            break;
        case MessageType::DesiredCourse:
        {
            DesiredCourseMsg* courseMsg = (DesiredCourseMsg*)msg;
            desiredHeading = courseMsg->desiredCourse();
            command = pi();
        }
            break;
        default:
            break;
    }

    if(command != 370)
    {
        // Send rudder commands
    }
}

///----------------------------------------------------------------------------------
uint16_t LowLevelController::pi()
{
    static double integral = 0;
    int16_t error = 0;

    if( heading == 370 ) { return 0; }
    if( desiredHeading == 370) { return heading; }

    error = Utility::headingDifference( heading, desiredHeading );

    // Prevent integral windup
    if( abs(error) > 15 )
    {
        integral = integral + error;
    }
    else 
    {
        integral = 0;
    }

    int16_t p = error * pGain;
    int16_t i = integral * iGain;

    return p + i;
}