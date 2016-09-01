/****************************************************************************************
 *
 * File:
 * 		WRSC.h
 *
 * Purpose:
 *		
 *
 * Developer Notes:
 *
 ***************************************************************************************/


#pragma once



#define BOAT_JANET			0
#define BOAT_ENSTA_GRAND	1
#define BOAT_ENSTA_PETIT	2

#define BOAT_TYPE			BOAT_ENSTA_GRAND


// HARD LIMITS (FOR WRSC ON ENSTA BOAT)



#if BOAT_TYPE == BOAT_ENSTA_GRAND

#define SAIL_MAX_US		1725
#define SAIL_MIN_US		1301

#define RUDDER_MAX_US	1857
#define RUDDER_MID_US	1369
#define RUDDER_MIN_US	962

#elif BOAT_TYPE == BOAT_ENSTA_PETIT

#define SAIL_MAX_US		1725
#define SAIL_MIN_US		1301

#define RUDDER_MAX_US	2000
#define RUDDER_MID_US	1500
#define RUDDER_MIN_US	1000

#endif
