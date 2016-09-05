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

#define BOAT_TYPE			BOAT_ENSTA_PETIT


// HARD LIMITS (FOR WRSC ON ENSTA BOAT)


#if BOAT_TYPE == BOAT_JANET

#define SAIL_MAX_US		2000 // DOUBLE CHECK THIS WITH CONFIG
#define SAIL_MIN_US		1000

#define RUDDER_MAX_US	2000
#define RUDDER_MID_US	1500
#define RUDDER_MIN_US	1000

#define MAX_RUDDER_COMMAND 		M_PI / 6 		// 29.9846
#define MAX_SAIL_COMMAND 		0.6958 			// 42 Degrees
#define MIN_SAIL_COMMAND 		M_PI / 32.0f;	// 5.6 Degrees
#define TACK_ANGLE				0.872665; 		//50°


el#if BOAT_TYPE == BOAT_ENSTA_GRAND

#define SAIL_MAX_US		1725
#define SAIL_MIN_US		1301

#define RUDDER_MAX_US	1857
#define RUDDER_MID_US	1369
#define RUDDER_MIN_US	962

#define MAX_RUDDER_COMMAND 		0.69813 		// 40 Degrees
#define MAX_SAIL_COMMAND 		1.22173 		// 75 Degrees
#define MIN_SAIL_COMMAND 		M_PI / 32.0f;	// 5.6 Degrees
#define TACK_ANGLE				0.7854; 		// 45°

#elif BOAT_TYPE == BOAT_ENSTA_PETIT

#define SAIL_MAX_US		1750
#define SAIL_MIN_US		1100

#define RUDDER_MAX_US	2000
#define RUDDER_MID_US	1500
#define RUDDER_MIN_US	1000

#define MAX_RUDDER_COMMAND 		0.7854 		// 45 Degrees
#define MAX_SAIL_COMMAND 		1.22173 	// 75 Degrees
#define MIN_SAIL_COMMAND 		0.04363; 	// 2.5 Degrees
#define TACK_ANGLE				0.872665; 	// 50°

#endif
