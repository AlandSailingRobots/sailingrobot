#ifndef __SAILINGROBOT_H__
#define __SAILINGROBOT_H__

#include <memory>

#include "dbhandler/DBHandler.h"
#include "dbhandler/JSON.h"
#include "sailcommand/SailCommand.h"
#include "ruddercommand/RudderCommand.h"
#include "coursecalculation/CourseCalculation.h"
#include "httpsync/HTTPSync.h"

#include "Compass/Compass.h"
#include "Compass/MockCompass.h"
#include "Compass/HMC6343.h"

#include "CV7/Windsensor.h"
#include "CV7/MockWindsensor.h"
#include "CV7/CV7.h"

#include "gps/GPSReader.h"
#include "gps/MockGPSReader.h"

#include "servocontroller/Actuator.h"
#include "servocontroller/MaestroController.h"
#include "servocontroller/MockMaestroController.h"

#include "servocontroller/ServoObject.h"
#include "servocontroller/MockServoObject.h"

#include "thread/ExternalCommand.h"
#include "thread/SystemState.h"

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

	int mockLongitude(int oldLong);
	int mockLatitude(int oldLat);

	void logMessage(std::string type, std::string message);
	//void readGPS();
	void syncServer();
	void updateState();
	void nextWaypoint();

	//void setupDB(std::string filename);
	void setupMaestro();
	void setupRudderServo();
	void setupSailServo();
	void setupWindSensor();
	//void setupGPS();
	void setupCourseCalculation();
	void setupRudderCommand();
	void setupSailCommand();
	void setupWaypoint();
	void setupHTTPSync();
	void setupCompass();

	std::string m_errorLogPath;

	float m_waypointLatitude;
	float m_waypointLongitude;
	std::string m_waypointId;

	bool m_running;

	/**
	 *  bool flags for signaling the use of mock objects
	 */
	bool m_mockWindsensor;
	bool m_mockCompass;
	bool m_mockPosition;
	bool m_mockMaestro;

	/* true  = get heading from compass
	 * false = get heading from gps	 */
	bool m_getHeadingFromCompass;

	DBHandler *m_dbHandler;
	RudderCommand m_rudderCommand;
	SailCommand m_sailCommand;
	CourseCalculation m_courseCalc;
	HTTPSync m_httpSync;

	Compass* m_compass;
	GPS* m_gpsReader;
	Windsensor* m_windSensor;

	std::unique_ptr<Actuator> m_maestroController;
	ServoObject m_rudderServo;
	ServoObject m_sailServo;

	ExternalCommand* m_externalCommand;
	SystemState *m_systemState;

	SystemStateModel m_systemStateModel;

};
#endif
