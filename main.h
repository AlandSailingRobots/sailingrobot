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
#include "xBee/xBeeSync.h"
#include "GPSupdater.h"
#include "WindsensorController.h"
#include "thread/ThreadRAII.h"
#include "logger/Logger.h"
#include "httpsync/HTTPSync.h"
#include "i2ccontroller/I2CController.h"

std::unique_ptr<SailingRobot> sr_handle;
std::unique_ptr<xBeeSync> xbee_handle;
GPSupdater* gps_handle;
std::unique_ptr<WindsensorController> windsensor_handle;
std::unique_ptr<HTTPSync> httpsync_handle;
std::unique_ptr<I2CController> i2cController_handle;

std::unique_ptr<ThreadRAII> windsensor_thread;
std::unique_ptr<ThreadRAII> httpsync_thread;
std::unique_ptr<ThreadRAII> i2cController_thread;

//HTTPSync *httpsync_handle;

Logger m_logger;

#endif /* MAIN_H_ */
