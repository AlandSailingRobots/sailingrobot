/*
 * global.h
 *
 *  Created on: May 20, 2015
 *      Author: sailbot
 */

#ifndef MAIN_H_
#define MAIN_H_

#include <memory>
#include <thread>
#include "SailingRobot.h"
#include "xBeeSync.h"
#include "GPSupdater.h"
#include "WindsensorController.h"
#include "thread/ThreadRAII.h"
#include "logger/Logger.h"

SailingRobot *sr_handle;
std::unique_ptr<xBeeSync> xbee_handle;
GPSupdater *gps_handle;
std::unique_ptr<WindsensorController> windsensor_handle;

ThreadRAII *windsensor_thread;

bool m_mockGPS;
bool m_xBeeOFF;
bool m_mockWindsensor;

Logger m_logger;

#endif /* MAIN_H_ */
