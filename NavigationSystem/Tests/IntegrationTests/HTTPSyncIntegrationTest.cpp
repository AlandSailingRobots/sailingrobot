#include "httpsync/HTTPSync.h"
#include "dbhandler/DBHandler.h"
#include "models/SystemStateModel.h"
#include <iostream>
#include <string>

int main() {

	DBHandler* db = new DBHandler("/home/sailbot/sailingrobot/asr.db");
	HTTPSync sync(db, 0, 0);

	/* init credentials*/
	sync.setShipID("BOAT01");
	sync.setShipPWD("PWD01");
	/*Pushing data towards hostgator*/
	sync.setServerURL("http://www.sailingrobots.com/testdata/sync/");

	//Need to do an insert in the HTTPsync run loop to make the test working properly :
	SystemStateModel systemStateModel = SystemStateModel(
			GPSModel("",PositionModel(0,0),0,0,0,0),
			WindsensorModel(0,0,0),
			CompassModel(0,0,0,AccelerationModel(0,0,0) ),
			AnalogArduinoModel(0, 0, 0, 0),
			0,
			0
		);

	db->insertDataLog(systemStateModel,1,1,1.0,1.0,1.0,true,true,1,1.0,0);

	try {
		sync.run();
	} catch(const char* error) {
		std::cout << error << std::endl;
	}

	delete db;
}
