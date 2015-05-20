/*
 * global.h
 *
 *  Created on: May 20, 2015
 *      Author: sailbot
 */

#ifndef GLOBAL_H_
#define GLOBAL_H_

#include "SailingRobot.h"
#include "xBeeSync.h"
#include "GPSupdater.h"
#include "dbhandler/DBHandler.h"

SailingRobot *sr_handle;
xBeeSync *xbee_handle;
GPSupdater *gps_handle;
DBHandler *db_handle;

#endif /* GLOBAL_H_ */
