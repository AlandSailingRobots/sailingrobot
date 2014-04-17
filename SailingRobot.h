#include "../windsensor/WindSensorController.h"
#include "../sailcommand/SailCommand.h"
#include "../ruddercommand/RudderCommand.h"
#include "../servocontroller/MaestroController.h"
#include "../servocontroller/ServoObject.h"
#include "../gps/GPSReader.h"
#include "../dbhandler/DBHandler.h"
#include "../coursecalculation/CourseCalculation.h"
#include "../waypointlist/WaypointList.h"

/*
class GPSReader {
//mock gpsreader
	public:
		GPSReader() {
	};
	~GPSReader() {
		std::cout << "gps detr\n";
	}
	void connectToGPS() {
	};
	void readGPS() {
	};
	string getTimestamp() {
		return "timestamp";
	};
	double getLatitude() {
		return 30.2;
	};
	double getLongitude() {
		return 40.9;
	};
	double getAltitude() {
		return 40.3;
	};
	double getSpeed() {
		return 3.04;
	};
	double getHeading() {
		return 230.4;
	};
};*/


class SailingRobot {

public:

	SailingRobot();
	~SailingRobot();
	void init();
	void run();
	void shutdown();

private:

	WindSensorController m_windSensorController;
	CourseCalculation m_courseCalc;

	MaestroController m_maestroController;
	ServoObject m_rudderServo;
	ServoObject m_sailServo;

	DBHandler m_dbHandler;

	WaypointList m_waypointList;
	GPSReader m_gpsReader;

	RudderCommand m_rudderCommand;
	SailCommand m_sailCommand;


};