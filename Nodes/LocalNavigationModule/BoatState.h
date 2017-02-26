/****************************************************************************************
 *
 * File:
 * 		BoatState.h
 *
 * Purpose:
 *		A cache the Local Navigation Module uses to store boat data.
 *
 * License:
 *      This file is subject to the terms and conditions defined in the file 
 *      'LICENSE.txt', which is part of this source code package.
 *
 ***************************************************************************************/


#pragma once


#include <stdint.h>


struct BoatState_t {
    double currWaypointLat;
    double currWaypointLon;
    double lastWaypointLat;
    double lastWaypointLon;

    uin16_t heading;
    double lat;
    double lon;
    uint16_t wind;
    double speed;
};