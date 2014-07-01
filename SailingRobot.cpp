#include "SailingRobot.h"
#include <cstdlib>
#include <iostream>
#include <wiringPi.h>
#include <unistd.h>
#include <fstream>
#include <cstring>

SailingRobot::SailingRobot() {
/*	wiringPiSetup();
	pinMode(6, OUTPUT);
	digitalWrite(6, HIGH);*/
}

SailingRobot::~SailingRobot() {
/*	digitalWrite(6, LOW);
	std::cout << "sr destruct\n";*/
}


void SailingRobot::init(string programPath, string dbFileName, string errorFileName) {

	m_errorLogPath = programPath + errorFileName;
	setupDB(programPath + dbFileName);
	logMessage("message", "setupDB() done");

	setupHTTPSync();
	logMessage("message", "setupHTTPSync() done"); syncServer();
	////////////////////////////////////////////////////////////////////////////
	std::cout << "sync setup done\n";
/*	try {
		//m_dbHandler.updateConfig(m_httpSync.getConfig());
		std::cout << m_httpSync.getRoute() << "\n";
		//m_dbHandler.updateWaypoints(m_httpSync.getRoute());
	} catch (const char * e) {
		std::cout << e << "\n";
	}*/
//	exit(0);
		throw "rtrtrtrt";

	////////////////////////////////////////////////////////////////////////////

	setupMaestro();
	logMessage("message", "setupMaestro() done"); syncServer();

	setupRudderServo();
	logMessage("message", "setupRudderServo() done"); syncServer();

	setupSailServo();
	logMessage("message", "setupSailServo() done"); syncServer();

	setupWindSensor();
	logMessage("message", "setupWindSensor() done"); syncServer();

	setupGPS();
	readGPS();
	while (isnan(m_gpsReader.getLatitude())) {
		readGPS();
		sleep(2);
	}
	logMessage("message", "setupGPS() done"); syncServer();

	setupCourseCalculation();
	logMessage("message", "setupCourseCalculation() done"); syncServer();

	setupRudderCommand();
	logMessage("message", "setupRudderCommand() done"); syncServer();

	setupSailCommand();
	logMessage("message", "setupSailCommand() done"); syncServer();

	setupWaypointList();
	logMessage("message", "setupWaypointList() done"); syncServer();
}


void SailingRobot::run() {

	int rudderCommand, sailCommand, windDir, twd;

	while(true) {

		//read windsensor
		windDir = m_windSensor.getDirection();

		if ( !isnan(m_gpsReader.getLatitude()) ) {

			//calc DTW
			m_courseCalc.calculateDTW(m_gpsReader.getLatitude(), m_gpsReader.getLongitude(),
				m_waypointList.getLatitude(), m_waypointList.getLongitude());

			//check if we are within 15meters of the waypoint and move to next wp in that case
			if (m_courseCalc.getDTW() < 15) {
				m_waypointList.next();
			}

			//calc & set TWD
			twd = m_gpsReader.getHeading() + windDir - 180;
			if (twd > 359) {
				twd -= 360;
			}
			if (twd < 0) {
				twd += 360;
			}
			m_courseCalc.setTWD(twd);

			//calc BTW & CTS
			m_courseCalc.calculateBTW(m_gpsReader.getLatitude(), m_gpsReader.getLongitude(),
				m_waypointList.getLatitude(), m_waypointList.getLongitude());
			m_courseCalc.calculateCTS();

			//rudder position calculation
			rudderCommand = m_rudderCommand.getCommand(m_courseCalc.getCTS(), m_gpsReader.getHeading());

		} else {
			logMessage("error", "SailingRobot::run(), gps NaN. Using values from last iteration.");
		}

		//sail position calculation
		sailCommand = m_sailCommand.getCommand(windDir);

		//rudder adjustment
		m_rudderServo.setPosition(rudderCommand);
		//sail adjustment
		m_sailServo.setPosition(sailCommand);

		//logging
		int sailServoPosition, rudderServoPosition;
		try {
			sailServoPosition = m_sailServo.getPosition();
		} catch (const char * error) {
			logMessage("error", error);
			sailServoPosition = 0;
		}
		try {
			rudderServoPosition = m_rudderServo.getPosition();
		} catch (const char * error) {
			logMessage("error", error);
			rudderServoPosition = 0;
		}

		m_dbHandler.insertDataLog(
			m_gpsReader.getTimestamp(),
			m_gpsReader.getLatitude(),
			m_gpsReader.getLongitude(),
			m_gpsReader.getSpeed(),
			m_gpsReader.getHeading(),
			m_gpsReader.getSatellitesUsed(),
			sailCommand,
			rudderCommand,
			sailServoPosition,
			rudderServoPosition,
			m_courseCalc.getDTW(),
			m_courseCalc.getBTW(),
			m_courseCalc.getCTS(),
			m_courseCalc.getTACK(),
			windDir,
			0,
			0,
			m_waypointList.getCurrent());

		syncServer();

		//update gps
		readGPS();
	}
}


void SailingRobot::shutdown() {
	m_dbHandler.closeDatabase();
}


void SailingRobot::logMessage(string type, string message) {
	try {
		m_dbHandler.insertMessageLog(m_gpsReader.getTimestamp(), type, message);
	} catch (const char * logError) {
		std::ofstream errorFile;
		errorFile.open(m_errorLogPath.c_str(), ios::app);
		errorFile << "log error: " << logError << "\n";
		errorFile << "when logging " << type << ": " << message << "\n";
		errorFile.close();
	}
}

void SailingRobot::readGPS() {
	try {
		m_gpsReader.readGPS(50000000); //microseconds
	} catch (const char * error) {
		logMessage("error", error);
	}
}

void SailingRobot::syncServer() {
	m_httpSync.pushLogs( m_dbHandler.getLogs() );
	m_dbHandler.clearTable("datalogs");
	m_dbHandler.clearTable("messages");
}




///////// setup crap
/////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////

void SailingRobot::setupDB(string filename) {
	try {
		m_dbHandler.openDatabase(filename);
	} catch (const char * error) {
		logMessage("error", error);
		throw;
	}
}

void SailingRobot::setupMaestro() {
	std::string val;
	try {
		std::string val = m_dbHandler.retriveCell("configs", "1", "mc_port");
		m_maestroController.setPort(val.c_str());
	} catch (const char * error) {
		logMessage("error", error);
		throw;
	}
}

void SailingRobot::setupRudderServo() {
	std::string val;
	try {
		m_rudderServo.setController(&m_maestroController);
		val = m_dbHandler.retriveCell("configs", "1", "rs_chan");
		m_rudderServo.setChannel(atoi(val.c_str()));
		val = m_dbHandler.retriveCell("configs", "1", "rs_spd");
		m_rudderServo.setSpeed(atoi(val.c_str()));
		val = m_dbHandler.retriveCell("configs", "1", "rs_acc");
		m_rudderServo.setAcceleration(atoi(val.c_str()));
	} catch (const char * error) {
		logMessage("error", error);
		throw;
	}
}

void SailingRobot::setupSailServo() {
	std::string val;
	try {
		m_sailServo.setController(&m_maestroController);
		val = m_dbHandler.retriveCell("configs", "1", "ss_chan");
		m_sailServo.setChannel(atoi(val.c_str()));
		val = m_dbHandler.retriveCell("configs", "1", "ss_spd");
		m_sailServo.setSpeed(atoi(val.c_str()));
		val = m_dbHandler.retriveCell("configs", "1", "ss_acc");
		m_sailServo.setAcceleration(atoi(val.c_str()));
	} catch (const char * error) {
		logMessage("error", error);
		throw;
	}
}

void SailingRobot::setupWindSensor() {
	std::string val;
	try {
		m_windSensor.setController(&m_maestroController);
		val = m_dbHandler.retriveCell("configs", "1", "ws_chan");
		m_windSensor.setChannel(atoi(val.c_str()));
	} catch (const char * error) {
		logMessage("error", error);
		throw;
	}
}

void SailingRobot::setupGPS() {
	try {
		m_gpsReader.connectToGPS();
	} catch (const char * error) {
		logMessage("error", error);
		throw;
	}	
}

void SailingRobot::setupCourseCalculation() {
	std::string val;
	try {
		val = m_dbHandler.retriveCell("configs", "1", "cc_ang_tack");
		m_courseCalc.setTACK_ANGLE(atoi(val.c_str()));
		val = m_dbHandler.retriveCell("configs", "1", "cc_ang_sect");
		m_courseCalc.setSECTOR_ANGLE(atoi(val.c_str()));
	} catch (const char * error) {
		logMessage("error", error);
		throw;
	}
}

void SailingRobot::setupRudderCommand() {
	std::string val, val2, val3, val4;
	try {
		val = m_dbHandler.retriveCell("configs", "1", "rc_cmd_xtrm");
		val2 = m_dbHandler.retriveCell("configs", "1", "rc_cmd_med");
		val3 = m_dbHandler.retriveCell("configs", "1", "rc_cmd_sml");
		val4 = m_dbHandler.retriveCell("configs", "1", "rc_cmd_mid");
		m_rudderCommand.setCommandValues(atoi(val.c_str()), atoi(val2.c_str()), atoi(val3.c_str()), atoi(val4.c_str()));
		val = m_dbHandler.retriveCell("configs", "1", "rc_ang_med");
		val2 = m_dbHandler.retriveCell("configs", "1", "rc_ang_sml");
		val3 = m_dbHandler.retriveCell("configs", "1", "rc_ang_mid");
		m_rudderCommand.setAngleValues(atoi(val.c_str()), atoi(val2.c_str()), atoi(val3.c_str()));
	} catch (const char * error) {
		logMessage("error", error);
		throw;
	}
}

void SailingRobot::setupSailCommand() {
	std::string val, val2, val3, val4;
	try {
		val = m_dbHandler.retriveCell("configs", "1", "sc_cmd_clse");
		val2 = m_dbHandler.retriveCell("configs", "1", "sc_cmd_beam");
		val3 = m_dbHandler.retriveCell("configs", "1", "sc_cmd_brd");
		val4 = m_dbHandler.retriveCell("configs", "1", "sc_cmd_run");
		m_sailCommand.setCommandValues(atoi(val.c_str()), atoi(val2.c_str()), atoi(val3.c_str()), atoi(val4.c_str()));
		val = m_dbHandler.retriveCell("configs", "1", "sc_ang_beam");
		val2 = m_dbHandler.retriveCell("configs", "1", "sc_ang_brd");
		val3 = m_dbHandler.retriveCell("configs", "1", "sc_ang_run");
		m_sailCommand.setAngleValues(atoi(val.c_str()), atoi(val2.c_str()), atoi(val3.c_str()));
	} catch (const char * error) {
		logMessage("error", error);
		throw;
	}
}

void SailingRobot::setupWaypointList() {
	m_waypointList.add(60.103580, 19.867784);
	m_waypointList.add(60.103677, 19.865273);
	m_waypointList.add(60.104489, 19.866818);
/*	std::string val, val2;
	try {
		val = m_dbHandler.retriveCell("waypoints", "1", "latitude");
		val2 = m_dbHandler.retriveCell("waypoints", "1", "longitude");
		m_waypointList.add(strtod(val.c_str(), NULL), strtod(val2.c_str(), NULL));
	} catch (const char * error) {
		logMessage("error", error);
		throw;
	}*/
}


void SailingRobot::setupHTTPSync() {
	std::string val;
	try {
		val = m_dbHandler.retriveCell("server", "1", "boat_id");
		m_httpSync.setShipID(val);
		val = m_dbHandler.retriveCell("server", "1", "boat_pwd");
		m_httpSync.setShipPWD(val);
		val = m_dbHandler.retriveCell("server", "1", "srv_addr");
		m_httpSync.setServerURL(val);
	} catch (const char * error) {
		logMessage("error", error);
		throw;
	}
}