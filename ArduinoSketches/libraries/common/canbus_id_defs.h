/****************************************************************************************
*
* File:
* 		canbus_id_defs.h
*
* Purpose:
*		 The purpose of this definitions is a unified use of ids from both Arduino and RPI
*
* Developer Notes:
*       The full specifications of these ids can be found at:
*       https://docs.google.com/spreadsheets/d/1TQ0txUN-F_I23I8oXek--Sg3eLZSvcd2x4Pli3p4F-Q/edit?usp=sharing
*
***************************************************************************************/


#ifndef SAILINGROBOT_CANBUS_ID_DEFS_H
#define SAILINGROBOT_CANBUS_ID_DEFS_H

/*
 *  Contains the following information:
 *  Rudder control, 2 bytes
 *  Wingsail Control, 2 bytes
 *  Windvane self steering, 2 bytes
 *  windvane self steering on/off, 1 byte
 *  Propeller speed, 1 byte
 */
#define MSG_ID_AU_CONTROL 700


/*
 *  Contains the following information:
 *  Rudder feedback, 2 bytes
 *  Wingsail feedback, 2 bytes
 *  Windvane self steering angle, 2 bytes
 *  Windvane self steering actuator position, 1 byte
 */
#define MSG_ID_AU_FEEDBACK 701


/*
 *  Contains the following information:
 *  latitude, 4 bytes
 *  longitude, 4 bytes
 */
#define MSG_ID_SOLAR_PANEL_CONTROL_PART_1 703


/*
 *  Contains the following information:
 *  time, 4 bytes
 *  heading, 4 bytes
 */
#define MSG_ID_SOLAR_PANEL_CONTROL_PART_2 704


/*
 *  Contains the following information:
 *  extended length, 2 bytes
 */
#define MSG_ID_WINCH_CONTROL 800


/*
 *  Contains the following information:
 *  extended length, 2 bytes
 */
#define MSG_ID_WINCH_FEEDBACK 801


/*
 *  Contains the following information:
 *  RC on/ off, 1 byte
 */
#define MSG_ID_RC_STATUS 702


/*
 *  Contains the following information:
 *  read sensors on interval on/off, 1 byte
 *  Sensor read interval, 4 byte integer
 */
#define MSG_ID_MARINE_SENSOR_REQUEST 710


/*
 *  Contains the following information:
 *  read sensors on interval on/off, 1 byte
 *  Sensor read interval, 4 byte integer
 *  Error message, 1 byte at last position
 */
#define MSG_ID_MARINE_SENSOR_DATA 711

#endif //SAILINGROBOT_CANBUS_ID_DEFS_H
