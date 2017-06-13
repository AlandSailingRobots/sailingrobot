/****************************************************************************************
 *
 * File:
 * 		LowLevelController.h
 *
 * Purpose:
 *
 *
 * License:
 *      This file is subject to the terms and conditions defined in the file
 *      'LICENSE.txt', which is part of this source code package.
 *
 ***************************************************************************************/


#pragma once


#include "MessageBus/Node.h"
#include "dbhandler/DBHandler.h"


class LowLevelController : public Node {
public:
    ///----------------------------------------------------------------------------------
 	/// Contructs a LowLevelController
 	///----------------------------------------------------------------------------------
    LowLevelController( MessageBus& msgBus, DBHandler& dbHandler, double configPGain, double configIGain );

    ///----------------------------------------------------------------------------------
 	/// Does nothing
 	///----------------------------------------------------------------------------------
    bool init() { return true; }

    ///----------------------------------------------------------------------------------
 	/// Processes DesiredCourseMsgs and compass data messages
 	///----------------------------------------------------------------------------------
    void processMessage( const Message* msg );
private:
    void sendActuatorMsg();
    void calculateRudder();
    void calculateSail( int windDir );
    int16_t restrictRudder( int16_t val );
    int16_t pi();

    uint16_t heading;
    uint16_t desiredHeading;
    double pGain;
    double iGain;

    // These are in maestro servo range
    uint16_t closeRange_ms;
    uint16_t sailRange_ms;
    uint16_t rudderMidpoint_ms;
    uint16_t rudderRange_ms;

    int16_t rudder_ms;
    int16_t sail_ms;


    const int HEADING_ERROR_VALUE = 370;

    const int MAX_RUDDER_ANGLE = 30;
};
