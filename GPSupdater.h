/*
 * GPSupdater.h
 *
 *  Created on: May 19, 2015
 *      Author: sailbot
 */

#ifndef GPS_GPSUPDATER_H_
#define GPS_GPSUPDATER_H_

#include <mutex>
#include "gps/GPS.h"

class GPSupdater {

	public:
		GPSupdater(SystemState *systemState, bool mockIt);
		~GPSupdater(){};
		void run();
		void close();

	private:
		GPS m_gpsReader;
		std::mutex m_mutex;
		bool m_running;

		bool isRunning();
};

#endif /* GPS_GPSUPDATER_H_ */
