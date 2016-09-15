/****************************************************************************************
 *
 * File:
 * 		PWMValues.h
 *
 * Purpose:
 *		This file contains the max and min PWM values for the rudder and sail actuators.
 *
 * Developer Notes:
 * 		Its worth noting that the Maestro doesn't accept raw PWM values but
 * 		quarter-microseconds. That is the PWM value * 4. This * 4 modifer is applied in
 * 		the actuator node
 *
 ***************************************************************************************/


#pragma once


#define RUDDER_MAX_US	1750
#define RUDDER_MIN_US	1010

#define SAIL_MAX_US		2000
#define SAIL_MIN_US		1000

