/****************************************************************************************
 *
 * File:
 * 		Collidable_t.h
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


#include <stdint.h>
#include <map> 


struct VisualField_t {
    std::map<int16_t, uint16_t> bearingToRelativeFreeDistance;
    int16_t visibleFieldLowBearingLimit;
    int16_t visibleFieldHighBearingLimit;
    unsigned long lastUpdated;
};

struct AISCollidable_t {
    uint32_t mmsi;
    float course;
    double latitude;
    double longitude;
    float speed;
    unsigned long lastUpdated;
    float length;
    float beam;
};
