#include "SailingRobot.h"
#include <cstdlib>
#include <iostream>
#include <wiringPi.h>
#include <unistd.h>
#include <fstream>
#include <cstring>

SailingRobot::SailingRobot() {
/*	sleepypi stuff
	wiringPiSetup();
	pinMode(6, OUTPUT);
	digitalWrite(6, HIGH);*/
}

SailingRobot::~SailingRobot() {
/*	sleepypi stuff
	digitalWrite(6, LOW);
	std::cout << "sr destruct\n";*/
}


void SailingRobot::init(std::string programPath, std::string dbFileName, std::string errorFileName) {
	m_errorLogPath = programPath + errorFileName;
	setupDB(programPath + dbFileName);

//	setupHTTPSync();

//	updateState();

	setupMaestro();  //syncServer();

	setupRudderServo(); //syncServer();

	setupSailServo(); //syncServer();

	setupWindSensor(); //syncServer();

	setupGPS();
	readGPS();
	while (isnan(m_gpsReader.getLatitude())) {
		readGPS();
	}
	//syncServer();

	setupCourseCalculation(); //syncServer();

	setupRudderCommand(); //syncServer();

	setupSailCommand(); //syncServer();

	setupWaypoint(); //syncServer();
}


void SailingRobot::run() {
	int rudderCommand, sailCommand, windDir, compDir, twd;

	while(!m_waypointId.empty()) {
std::cout << "main loop iteration\n";
		//read windsensor
		m_windSensor.refreshData();
		windDir = m_windSensor.getDirection();

		m_Compass.readHeading();
		compDir = m_Compass.getHeading();

		if ( !isnan(m_gpsReader.getLatitude()) ) {

			//calc DTW
			m_courseCalc.calculateDTW(m_gpsReader.getLatitude(), m_gpsReader.getLongitude(),
				m_waypointLatitude, m_waypointLongitude);

			//calc & set TWD
			twd = m_gpsReader.getHeading() + windDir;
			if (twd > 359) {
				twd -= 360;
			}
			if (twd < 0) {
				twd += 360;
			}
			m_courseCalc.setTWD(twd);

			//calc BTW & CTS
			m_courseCalc.calculateBTW(m_gpsReader.getLatitude(), m_gpsReader.getLongitude(),
				m_waypointLatitude, m_waypointLongitude);
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
		m_dbHandler.insertDataLog(
			m_gpsReader.getTimestamp(),
			m_gpsReader.getLatitude(),
			m_gpsReader.getLongitude(),
			m_gpsReader.getSpeed(),
			m_gpsReader.getHeading(),
			m_gpsReader.getSatellitesUsed(),
			sailCommand,
			rudderCommand,
			0, //sailservo getpos, to remove
			0, //rudderservo getpos, to remove
			m_courseCalc.getDTW(),
			m_courseCalc.getBTW(),
			m_courseCalc.getCTS(),
			m_courseCalc.getTACK(),
			windDir,
			m_windSensor.getSpeed(),
			m_windSensor.getTemperature(),
			atoi(m_waypointId.c_str()),
			m_Compass.getHeading(),
			0,//Pitch
			0//Roll
		);

//		syncServer();

		//update gps
		readGPS();

		//check if we are within 15meters of the waypoint and move to next wp in that case
		if (m_courseCalc.getDTW() < 15) {
			nextWaypoint();
			setupWaypoint();
		}
	}
}


void SailingRobot::shutdown() {
//	syncServer();
	m_dbHandler.closeDatabase();
}


void SailingRobot::logMessage(std::string type, std::string message) {
	try {
		m_dbHandler.insertMessageLog(m_gpsReader.getTimestamp(), type, message);
       	} catch (const char * logError) {
		std::ofstream errorFile;
		errorFile.open(m_errorLogPath.c_str(), std::ios::app);
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
	try {
		std::string response = m_httpSync.pushLogs( m_dbHandler.getLogs() );
		m_dbHandler.removeLogs(response);
	} catch (const char * error) {
		logMessage("error", error);
	}
}


void SailingRobot::updateState() {
	try {
		std::string setup = m_httpSync.getSetup();
		bool stateChanged = false;
		if (m_dbHandler.revChanged("cfg_rev", setup) ) {
			m_dbHandler.updateTable("configs", m_httpSync.getConfig());
			stateChanged = true;
			logMessage("message", "config state updated");
		}
		if (m_dbHandler.revChanged("rte_rev", setup) ) {
			m_dbHandler.updateTable("waypoints", m_httpSync.getRoute());
			stateChanged = true;
			logMessage("message", "route state updated");
		}
		if (stateChanged)  {
			m_dbHandler.updateTable("state", m_httpSync.getSetup());
		}
	} catch (const char * error) {
		logMessage("error", error);
	}
}


void SailingRobot::nextWaypoint() {
	try {
//		m_dbHandler.deleteRow("waypoints", m_waypointId);
	} catch (const char * error) {
		logMessage("error", error);
	}
	logMessage("message", "SailingRobot::nextWaypoint(), waypoint reached");
}


///////// setup crap
/////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////

void SailingRobot::setupDB(std::string filename) {
	try {
		m_dbHandler.openDatabase(filename);
	} catch (const char * error) {
		logMessage("error", error);
		throw;
	}
	logMessage("message", "setupDB() done");
}

void SailingRobot::setupMaestro() {
	try {
		m_maestroController.setPort( m_dbHandler.retriveCell("configs", "1", "mc_port") );
		int maestroErrorCode = m_maestroController.getError();
		if (maestroErrorCode != 0) {
			std::stringstream stream;
			stream << "maestro errorcode: " << maestroErrorCode;
			logMessage("error", stream.str());
		}
	} catch (const char * error) {
		logMessage("error", error);
		throw;
	}
	logMessage("message", "setupMaestro() done");
}

void SailingRobot::setupRudderServo() {
	try {
		m_rudderServo.setController(&m_maestroController);
		m_rudderServo.setChannel( m_dbHandler.retriveCellAsInt("configs", "1", "rs_chan") );
		m_rudderServo.setSpeed( m_dbHandler.retriveCellAsInt("configs", "1", "rs_spd") );
		m_rudderServo.setAcceleration( m_dbHandler.retriveCellAsInt("configs", "1", "rs_acc") );
	} catch (const char * error) {
		logMessage("error", error);
		throw;
	}
	logMessage("message", "setupRudderServo() done");
}

void SailingRobot::setupSailServo() {
	try {
		m_sailServo.setController(&m_maestroController);
		m_sailServo.setChannel( m_dbHandler.retriveCellAsInt("configs", "1", "ss_chan") );
		m_sailServo.setSpeed( m_dbHandler.retriveCellAsInt("configs", "1", "ss_spd") );
		m_sailServo.setAcceleration( m_dbHandler.retriveCellAsInt("configs", "1", "ss_acc") );
	} catch (const char * error) {
		logMessage("error", error);
		throw;
	}
	logMessage("message", "setupSailServo() done");
}

void SailingRobot::setupWindSensor() {
	try {
		m_windSensor.loadConfig( m_dbHandler.retriveCell("configs", "1", "ws_port"),
					m_dbHandler.retriveCellAsInt("configs", "1", "ws_baud") );

		m_windSensor.setBufferSize( m_dbHandler.retriveCellAsInt("configs", "1", "ws_buff") );
	} catch (const char * error) {
		logMessage("error", error);
		throw error;
	}
	logMessage("message", "setupWindSensor() done");
}

void SailingRobot::setupGPS() {
	try {
		m_gpsReader.connectToGPS();
	} catch (const char * error) {
		logMessage("error", error);
		throw;
	}
	logMessage("message", "setupGPS() done");
}

void SailingRobot::setupCourseCalculation() {
	try {
		m_courseCalc.setTACK_ANGLE( m_dbHandler.retriveCellAsInt("configs", "1", "cc_ang_tack") );
		m_courseCalc.setSECTOR_ANGLE( m_dbHandler.retriveCellAsInt("configs", "1", "cc_ang_sect") );
	} catch (const char * error) {
		logMessage("error", error);
		throw;
	}
	logMessage("message", "setupCourseCalculation() done");
}

void SailingRobot::setupRudderCommand() {
	try {
		m_rudderCommand.setCommandValues( m_dbHandler.retriveCellAsInt("configs", "1", "rc_cmd_xtrm"),
			m_dbHandler.retriveCellAsInt("configs", "1", "rc_cmd_mid"));

	} catch (const char * error) {
		logMessage("error", error);
		throw;
	}
	logMessage("message", "setupRudderCommand() done");
}

void SailingRobot::setupSailCommand() {
	try {
		m_sailCommand.setCommandValues( m_dbHandler.retriveCellAsInt("configs", "1", "sc_cmd_clse"),
			m_dbHandler.retriveCellAsInt("configs", "1", "sc_cmd_run"));

	} catch (const char * error) {
		logMessage("error", error);
		throw;
	}
	logMessage("message", "setupSailCommand() done");
}

void SailingRobot::setupWaypoint() {
	try {
		m_waypointId = m_dbHandler.getMinIdFromTable("waypoints");
		if (m_waypointId.empty()) {
			return;
		}

		std::string lat = m_dbHandler.retriveCell("waypoints", m_waypointId, "lat");
		m_waypointLatitude = atof(lat.c_str());
		std::string lon = m_dbHandler.retriveCell("waypoints", m_waypointId, "lon");
		m_waypointLongitude = atof(lon.c_str());
	} catch (const char * error) {
		logMessage("error", error);
		throw;
	}
	logMessage("message", "setupWaypoint() done");
}


void SailingRobot::setupHTTPSync() {
	try {
		m_httpSync.setShipID( m_dbHandler.retriveCell("server", "1", "boat_id") );
		m_httpSync.setShipPWD( m_dbHandler.retriveCell("server", "1", "boat_pwd") );
		m_httpSync.setServerURL( m_dbHandler.retriveCell("server", "1", "srv_addr") );
	} catch (const char * error) {
		logMessage("error", "SailingRobot::setupHTTPSync() failed");
	}
	logMessage("message", "setupHTTPSync() done");
}

void SailingRobot::setupCompass() {
	try {
		m_Compass.init();
	} catch (const char * error) {
		logMessage("error", "SailingRobot::setupCompass() failed");
	}
	logMessage("message", "setupCompass() done");
}
