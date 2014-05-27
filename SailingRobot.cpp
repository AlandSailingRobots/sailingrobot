#include "SailingRobot.h"
#include <iostream>
#include <wiringPi.h>

SailingRobot::SailingRobot() {
	wiringPiSetup();
	pinMode(6, OUTPUT);
	digitalWrite(6, HIGH);
}

SailingRobot::~SailingRobot() {
	digitalWrite(6, LOW);
	std::cout << "sr destruct\n";
}


void SailingRobot::init() {

	std::string val, val2, val3, val4;

	//dbhandler
	m_dbHandler.openDatabase("db");
std::cout << "dbh inited\n";

	//servos
	val = m_dbHandler.retriveCell("configs", "1", "mc_portname");
	m_maestroController.setPort(val.c_str());
std::cout << "maestro inited\n";

	m_rudderServo.setController(&m_maestroController);
	val = m_dbHandler.retriveCell("configs", "1", "rs_channel");
	m_rudderServo.setChannel(std::stoi(val));
	val = m_dbHandler.retriveCell("configs", "1", "rs_limitmin");
	m_rudderServo.setMin(std::stoi(val));
	val = m_dbHandler.retriveCell("configs", "1", "rs_limitmax");
	m_rudderServo.setMax(std::stoi(val));
	val = m_dbHandler.retriveCell("configs", "1", "rs_speed");
	m_rudderServo.setSpeed(std::stoi(val));
	val = m_dbHandler.retriveCell("configs", "1", "rs_acceleration");
	m_rudderServo.setAcceleration(std::stoi(val));

	m_sailServo.setController(&m_maestroController);
	val = m_dbHandler.retriveCell("configs", "1", "ss_channel");
	m_sailServo.setChannel(std::stoi(val));
	val = m_dbHandler.retriveCell("configs", "1", "ss_limitmin");
	m_sailServo.setMin(std::stoi(val));
	val = m_dbHandler.retriveCell("configs", "1", "ss_limitmax");
	m_sailServo.setMax(std::stoi(val));
	val = m_dbHandler.retriveCell("configs", "1", "ss_speed");
	m_sailServo.setSpeed(std::stoi(val));
	val = m_dbHandler.retriveCell("configs", "1", "ss_acceleration");
	m_sailServo.setAcceleration(std::stoi(val));

	m_windSensor.setController(&m_maestroController);
	m_windSensor.setChannel(5);

	void setCommandValues(int extreme, int medium, int small, int midships);

	// sets the angles used by getCommand() to return appropriate command,
	// extreme angle uses whats left over
	void setAngleValues(int medium, int small, int midships);
	std::cout << "servos inited\n";
	//windsensor
/*	try {
		m_windSensorController.loadConfig("CV7", "/dev/ttyAMA0", 4800);
	} catch(const char* exception) {
		cout << exception << endl;
		return;
	}
	//m_windSensorController.mockDirection(180);
std::cout << "windsens inited\n";*/



	//gps
	val = m_dbHandler.retriveCell("configs", "1", "gps_portname");
	val2 = m_dbHandler.retriveCell("configs", "1", "gps_connectionname");
	m_gpsReader.connectToGPS(val.c_str(), val2.c_str());
std::cout << "gpsr inited\n";

	//coursecalc
	val = m_dbHandler.retriveCell("configs", "1", "cc_tackangle");
	m_courseCalc.setTACK_ANGLE(std::stoi(val));
	val = m_dbHandler.retriveCell("configs", "1", "cc_sectorangle");
	m_courseCalc.setSECTOR_ANGLE(std::stoi(val));
std::cout << "coursecalc inited\n";


	val = m_dbHandler.retriveCell("configs", "1", "rc_commandextreme");
	val2 = m_dbHandler.retriveCell("configs", "1", "rc_commandmedium");
	val3 = m_dbHandler.retriveCell("configs", "1", "rc_commandsmall");
	val4 = m_dbHandler.retriveCell("configs", "1", "rc_commandmidships");
	m_rudderCommand.setCommandValues(std::stoi(val), std::stoi(val2), std::stoi(val3), std::stoi(val4));
	val = m_dbHandler.retriveCell("configs", "1", "rc_anglemedium");
	val2 = m_dbHandler.retriveCell("configs", "1", "rc_anglesmall");
	val3 = m_dbHandler.retriveCell("configs", "1", "rc_anglemidships");
	m_rudderCommand.setAngleValues(std::stoi(val), std::stoi(val2), std::stoi(val3));
std::cout << "ruddercommand inited\n";

	val = m_dbHandler.retriveCell("configs", "1", "sc_commandclosereach");
	val2 = m_dbHandler.retriveCell("configs", "1", "sc_commandbeamreach");
	val3 = m_dbHandler.retriveCell("configs", "1", "sc_commandbroadreach");
	val4 = m_dbHandler.retriveCell("configs", "1", "sc_commandrunning");
	m_sailCommand.setCommandValues(std::stoi(val), std::stoi(val2), std::stoi(val3), std::stoi(val4));
	val = m_dbHandler.retriveCell("configs", "1", "sc_anglebeamreach");
	val2 = m_dbHandler.retriveCell("configs", "1", "sc_anglebroadreach");
	val3 = m_dbHandler.retriveCell("configs", "1", "sc_anglerunning");
	m_sailCommand.setAngleValues(std::stoi(val), std::stoi(val2), std::stoi(val3));
std::cout << "sailcommand inited\n";


	//waypoints
	val = m_dbHandler.retriveCell("waypoints", "1", "latitude");
	val2 = m_dbHandler.retriveCell("waypoints", "1", "longitude");
	m_waypointList.add(std::stod(val), std::stod(val2));
std::cout << "wp inited\n";
}


void SailingRobot::run() {
	
	for (int i = 0; i < 100; i++) {

		//check sensors
		m_gpsReader.readGPS();
		try {
			m_windSensorController.refreshData();
		} catch(const char* exception) {
			cout << exception << endl;
		}

		//do coursecalc
		m_courseCalc.setTWD(m_windSensorController.getWindDirection());
		if (m_courseCalc.getDTW() < 15) {
			m_waypointList.next();
		}

		m_courseCalc.calculateBTW(m_gpsReader.getLatitude(), m_gpsReader.getLongitude(),
			m_waypointList.getLatitude(), m_waypointList.getLongitude());
		m_courseCalc.calculateDTW(m_gpsReader.getLatitude(), m_gpsReader.getLongitude(),
			m_waypointList.getLatitude(), m_waypointList.getLongitude());

		m_courseCalc.calculateCTS();


		//rudder adjust
        int rudderCommand = m_rudderCommand.getCommand(m_courseCalc.getCTS(), m_gpsReader.getHeading());
		m_rudderServo.setPosition(rudderCommand);


		//sail adjust
        int sailCommand = m_sailCommand.getCommand(m_windSensorController.getWindDirection());
		m_sailServo.setPosition(sailCommand);


		m_dbHandler.insertDataLog(sailCommand, rudderCommand, m_courseCalc.getDTW(), m_courseCalc.getBTW(),
		m_courseCalc.getCTS(), m_courseCalc.getTACK(),
		m_windSensorController.getBufferSize(), m_windSensorController.getSensorModel(),
		m_windSensorController.getWindDirection(), m_windSensorController.getWindSpeed(), 
		m_windSensorController.getWindTemperature(), m_rudderServo.getPosition(), m_sailServo.getPosition(),
		m_gpsReader.getTimestamp(), m_gpsReader.getLatitude(), m_gpsReader.getLongitude(),
		m_gpsReader.getAltitude(), m_gpsReader.getSpeed(), m_gpsReader.getHeading(),
		m_gpsReader.getMode(), m_gpsReader.getSatellites_used());




	}

}


void SailingRobot::shutdown() {
	m_dbHandler.closeDatabase();
}
