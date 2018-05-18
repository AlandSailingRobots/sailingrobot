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

// Describes the visual field
// a map containing a key for each bearing degree of the field of view and a value between
// 0 and 100, where 0 means an obstacle close at this bearing and 100 means no visible obstacle
// bearings are absolute bearings
struct VisualField_t {
    std::map<int16_t, uint16_t> bearingToRelativeObstacleDistance;
    std::map<int16_t, unsigned long> bearingToLastUpdated;
    int16_t visualFieldLowBearing;
    int16_t visualFieldHighBearing;
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
