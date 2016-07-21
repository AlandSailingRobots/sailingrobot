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

GPSupdater* gps_handle;
std::unique_ptr<WindsensorController> windsensor_handle;
HTTPSync* httpsync_handle;

std::unique_ptr<ThreadRAII> windsensor_thread;
std::unique_ptr<ThreadRAII> httpsync_thread;


#endif /* MAIN_H_ */
