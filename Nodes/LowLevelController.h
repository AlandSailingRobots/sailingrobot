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


#include "Node.h"


class LowLevelController : public Node {
public:
    ///----------------------------------------------------------------------------------
 	/// Contructs a LowLevelController
 	///----------------------------------------------------------------------------------
    LowLevelController( MessageBus& msgBus, double configPGain, double configIGain );

    ///----------------------------------------------------------------------------------
 	/// Does nothing
 	///----------------------------------------------------------------------------------
    bool init() { return true; }

    ///----------------------------------------------------------------------------------
 	/// Processes DesiredCourseMsgs and compass data messages
 	///----------------------------------------------------------------------------------
    void processMessage( const Message* msg );
private:
    uint16_t pi();

    uint16_t heading;
    uint16_t desiredHeading;
    double pGain;
    double iGain;
};