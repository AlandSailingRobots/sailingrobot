/*
 * global.h
 *
 *  Created on: May 20, 2015
 *      Author: sailbot
 */

#ifndef MAIN_H_
#define MAIN_H_

#include <memory>
#include "SailingRobot.h"
#include "xBeeSync.h"
#include "GPSupdater.h"

SailingRobot *sr_handle;
std::unique_ptr<xBeeSync> xbee_handle;
GPSupdater *gps_handle;

bool m_mockGPS;
bool m_xBeeOFF;

#endif /* MAIN_H_ */
