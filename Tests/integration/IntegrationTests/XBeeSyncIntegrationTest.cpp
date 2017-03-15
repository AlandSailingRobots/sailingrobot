#include "xBee/xBeeSync.h"
#include <iostream>
#include <string>
#include "xBee/xBee.h"
#include "xmlparser/src/xml_log.h"
#include "thread/ExternalCommand.h"
#include "thread/SystemState.h"
#include "models/SystemStateModel.h"
#include "logger/Logger.h"
#include "dbhandler/DBHandler.h"
#include "SystemServices/Timer.h"
#include <mutex>

int main() {

	DBHandler db("/home/sailbot/sailingrobot/asr.db");

	/*"Mock" system state model*/
	SystemStateModel systemStateModel = SystemStateModel(
			GPSModel("",PositionModel(0,0),0,0,0,0),
			WindsensorModel(0,0,0),
			CompassModel(0,0,0,AccelerationModel(0,0,0) ),
			AnalogArduinoModel(0, 0, 0, 0),
			0,
			0
		);

	SystemState systemState(systemStateModel);

	ExternalCommand externalCommand("<timestamp>", false, 0, 0);


	try {
		xBeeSync sync(&externalCommand, &systemState, &db, false, true, true, 0.2);
		/*Insert mock data into db*/
		db.insertDataLog(systemStateModel,1,1,1.0,1.0,1.0,true,true,1,1.0,0);
		sync.run();
	} catch(const char* error) {
		std::cout << error << std::endl;
	}

	/*Uncomment the run section to test integeration*/
	/*This test aims to push one mock datalog up to the server*/


}
