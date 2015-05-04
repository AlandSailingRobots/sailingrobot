#include "dbhandler/DBHandler.h"
#include "dbhandler/JSON.h"
#include "sailcommand/SailCommand.h"
#include "ruddercommand/RudderCommand.h"
#include "coursecalculation/CourseCalculation.h"
#include "httpsync/HTTPSync.h"

#include "Compass/Compass.h"
#include "Compass/MockCompass.h"

#include "CV7/Windsensor.h"
#include "CV7/MockWindsensor.h"

#include "gps/GPSReader.h"
#include "gps/MockGPSReader.h"

#include "servocontroller/MaestroController.h"
#include "servocontroller/MockMaestroController.h"

#include "servocontroller/ServoObject.h"
#include "servocontroller/MockServoObject.h"

#include "xmlparser/src/xml_log.h"

#include "xBee/xBee.h"


class SailingRobot {

public:

	SailingRobot();
	~SailingRobot();
	void init(std::string programPath, std::string dbFileName, std::string errorFileName);
	void run();
	void shutdown();

private:

	void logMessage(std::string type, std::string message);
	void readGPS();
	void syncServer();
	void updateState();
	void nextWaypoint();

	void setupDB(std::string filename);
	void setupMaestro();
	void setupRudderServo();
	void setupSailServo();
	void setupWindSensor();
	void setupGPS();
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

	DBHandler m_dbHandler;
	RudderCommand m_rudderCommand;
	SailCommand m_sailCommand;
	CourseCalculation m_courseCalc;
	HTTPSync m_httpSync;

	MockCompass m_Compass;
	MockGPSReader m_gpsReader;
	MockWindsensor m_windSensor;
	MockMaestroController m_maestroController;
	MockServoObject m_rudderServo;
	MockServoObject m_sailServo;

	XML_log m_XML_log;

	xBee m_xBee;
	int m_fd;
};
