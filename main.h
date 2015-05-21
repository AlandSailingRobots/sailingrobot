/*
 * global.h
 *
 *  Created on: May 20, 2015
 *      Author: sailbot
 */

#ifndef MAIN_H_
#define MAIN_H_

#include "SailingRobot.h"
#include "xBeeSync.h"
#include "GPSupdater.h"

SailingRobot *sr_handle;
xBeeSync *xbee_handle;
GPSupdater *gps_handle;

#endif /* MAIN_H_ */
