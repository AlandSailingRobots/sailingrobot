#include "windsensor/WindSensorController.h"
#include "sailcommand/SailCommand.h"
#include "ruddercommand/RudderCommand.h"
#include "servocontroller/MaestroController.h"
#include "servocontroller/ServoObject.h"
#include "servocontroller/SensorObject.h"
#include "gps/GPSReader.h"
#include "gps/MockGPSReader.h"
#include "dbhandler/DBHandler.h"
#include "dbhandler/JSON.h"
#include "coursecalculation/CourseCalculation.h"
#include "waypointlist/WaypointList.h"
#include "httpsync/HTTPSync.h"




class SailingRobot {

public:

	SailingRobot();
	~SailingRobot();
	void init(string programPath, string dbFileName, string errorFileName);
	void run();
	void shutdown();

private:

	void logMessage(string type, string message);
	void readGPS();
	void syncServer();
	void getServer();

	void setupDB(string filename);
	void setupMaestro();
	void setupRudderServo();
	void setupSailServo();
	void setupWindSensor();
	void setupGPS();
	void setupCourseCalculation();
	void setupRudderCommand();
	void setupSailCommand();
	void setupWaypointList();
	void setupHTTPSync();


	WindSensorController m_windSensorController;
	CourseCalculation m_courseCalc;

	MaestroController m_maestroController;
	ServoObject m_rudderServo;
	ServoObject m_sailServo;
	SensorObject m_windSensor;

	DBHandler m_dbHandler;

	WaypointList m_waypointList;
	MockGPSReader m_gpsReader;

	RudderCommand m_rudderCommand;
	SailCommand m_sailCommand;

	HTTPSync m_httpSync;

	std::string m_errorLogPath;
};
