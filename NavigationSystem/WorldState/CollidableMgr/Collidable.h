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


struct VisualCollidable_t {
    uint16_t n;
    uint16_t bearing;
    int lastUpdated;
};

struct AISCollidable_t {
    uint32_t mmsi;
    float course;
    double latitude;
    double longitude;
    float speed;
    int lastUpdated;
    float length;
    float beam;
};
