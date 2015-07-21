#ifndef __SAILINGROBOT_H__
#define __SAILINGROBOT_H__

#include <memory>

#include "dbhandler/DBHandler.h"
#include "sailcommand/SailCommand.h"
#include "ruddercommand/RudderCommand.h"
#include "coursecalculation/CourseCalculation.h"
#include "httpsync/HTTPSync.h"

#include "Compass/Compass.h"
#include "gps/GPS.h"

#include "servocontroller/Actuator.h"
#include "servocontroller/ServoObject.h"

#include "thread/ExternalCommand.h"
#include "thread/SystemState.h"

#include "logger/Logger.h"

#include "waypointrouting/WaypointRouting.h"

//#include "models/WaypointModel.h"

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

	double mockLongitude(double oldLong);
	double mockLatitude(double oldLat);

	//void readGPS();
	void syncServer();
	void updateState();
	void nextWaypoint();
	void setupWaypoint();

	//void setupDB(std::string filename);
	void setupMaestro();
	void setupRudderServo();
	void setupSailServo();
	//void setupGPS();
	void setupRudderCommand();
	void setupSailCommand();	
	void setupHTTPSync();
	void setupCompass();

	std::string m_errorLogPath;


	

	bool m_running;

	/**
	 *  bool flags for signaling the use of mock objects
	 */
	bool m_mockCompass;
	bool m_mockPosition;
	bool m_mockMaestro;

	/* true  = get heading from compass
	 * false = get heading from gps	 */
	bool m_getHeadingFromCompass;

	DBHandler *m_dbHandler;
	WaypointModel m_waypointModel;
	WaypointRouting m_waypointRouting;
	RudderCommand m_rudderCommand;
	SailCommand m_sailCommand;
	HTTPSync m_httpSync;

	Compass* m_compass;
	GPS* m_gpsReader;

	std::unique_ptr<Actuator> m_maestroController;
	ServoObject m_rudderServo;
	ServoObject m_sailServo;

	ExternalCommand* m_externalCommand;
	SystemState *m_systemState;

	SystemStateModel m_systemStateModel;

	Logger m_logger;
};

#endif