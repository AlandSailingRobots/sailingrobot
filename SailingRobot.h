#ifndef __SAILINGROBOT_H__
#define __SAILINGROBOT_H__

#include <memory>

#include "dbhandler/DBHandler.h"
#include "sailcommand/SailCommand.h"
#include "ruddercommand/RudderCommand.h"
#include "coursecalculation/CourseCalculation.h"
//#include "httpsync/HTTPSync.h"

#include "Compass/Compass.h"
#include "gps/GPS.h"

#include "servocontroller/Actuator.h"
#include "servocontroller/ServoObject.h"

#include "thread/ExternalCommand.h"
#include "thread/SystemState.h"

#include "logger/Logger.h"

#include "waypointrouting/WaypointRouting.h"
#include "windvanecontroller/WindVaneController.h"
//#include "models/WaypointModel.h"
#include "utility/RealPosition.h"
#include "utility/MockPosition.h"

class SailingRobot {

public:

	SailingRobot(ExternalCommand* externalCommand, SystemState *systemState, DBHandler *db);
	~SailingRobot();
	void init(std::string programPath, std::string errorFileName);
	void run();
	void shutdown();

	//void readGPS();

private:
	int getHeading();
	//Calculates a smooth transition between the compass and the gps. Do not call directly, use getHeading()
	int getMergedHeading(bool increaseCompassWeight);

	//void readGPS();
	//void syncServer();
	//void updateState();

	//void setupDB(std::string filename);
	void setupMaestro();
	void setupRudderServo();
	void setupSailServo();
	//void setupGPS();
	void setupRudderCommand();
	void setupSailCommand();
	//void setupHTTPSync();

	std::string m_errorLogPath;

	bool m_running;
	bool routeStarted;
	/**
	 *  bool flags for signaling the use of mock objects
	 */
	bool m_mockPosition;
	bool m_mockMaestro;

	/* true  = get heading from compass
	 * false = get heading from gps	 */
	bool m_getHeadingFromCompass;
	float m_gpsHeadingWeight;

	DBHandler *m_dbHandler;
	WindVaneController m_windVaneController;
	RudderCommand m_rudderCommand;
	SailCommand m_sailCommand;
	//HTTPSync m_httpSync;

	std::shared_ptr<Actuator> m_maestroController;
	ServoObject m_rudderServo;
	ServoObject m_sailServo;

	ExternalCommand* m_externalCommand;
	SystemState *m_systemState;
        std::unique_ptr<Position> position;

	SystemStateModel m_systemStateModel;

	Logger m_logger;
};

#endif
