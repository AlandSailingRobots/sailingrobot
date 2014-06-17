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


void SailingRobot::init() {

	setupDB("/root/sailingrobot/asr.db");
	setupMaestro();
	setupRudderServo();
	setupSailServo();
	setupWindSensor();
	setupGPS();
	readGPS();
	while (isnan(m_gpsReader.getLatitude())) {
		m_rudderServo.setPosition(m_rudderCommand.getMidShipsCommand());
		readGPS();
		sleep(2);
		m_rudderServo.setPosition(m_rudderCommand.getPortCommand());
		sleep(2);
	}	
	setupCourseCalculation();
	setupRudderCommand();
	setupSailCommand();
	setupWaypointList();
	setupHTTPSync();
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

			logError("SailingRobot::run(), gps NaN");
		}

		//sail position calculation
		sailCommand = m_sailCommand.getCommand(windDir);

		//rudder adjustment
		m_rudderServo.setPosition(rudderCommand);
		//sail adjustment
		m_sailServo.setPosition(sailCommand);

		//logging
		try {
			m_dbHandler.insertDataLog(
				m_gpsReader.getTimestamp(),
				m_gpsReader.getLatitude(),
				m_gpsReader.getLongitude(),
				m_gpsReader.getSpeed(),
				m_gpsReader.getHeading(),
				m_gpsReader.getSatellitesUsed(),
				sailCommand,
				rudderCommand,
				m_sailServo.getPosition(),
				m_rudderServo.getPosition(),
				m_courseCalc.getDTW(),
				m_courseCalc.getBTW(),
				m_courseCalc.getCTS(),
				m_courseCalc.getTACK(),
				windDir,
				0,
				0,
				m_waypointList.getCurrent());

		} catch (const char * error) {
			logError(error);
		}

		syncServer();

		//update gps
		readGPS();
	}
}


void SailingRobot::shutdown() {
	m_dbHandler.closeDatabase();
}


void SailingRobot::logError(string error) {
	try {
		m_dbHandler.insertMessageLog("timeplz", "error", error, 99);
	} catch (const char * logError) {
		std::ofstream errorFile;
			errorFile.open("/root/sailingrobot/errors.log", ios::app);
			errorFile << "log error: " << logError << "\n";
			errorFile << "when logging error: " << error << "\n";
		errorFile.close();
	}
}

void SailingRobot::readGPS() {
		try {
			m_gpsReader.readGPS(50000000);
		} catch (const char * error) {
			logError(error);
		}
}

void SailingRobot::setupDB(string filename) {
	try {
		m_dbHandler.openDatabase(filename);
	} catch (const char * error) {
		logError(error);
		exit(1);
	}
}

void SailingRobot::setupMaestro() {
	std::string val;
	try {
		std::string val = m_dbHandler.retriveCell("configs", "1", "mc_port");
		m_maestroController.setPort(val.c_str());
	} catch (const char * error) {
		logError(error);
		exit(1);
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
		logError(error);
		exit(1);
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
		logError(error);
		exit(1);
	}
}

void SailingRobot::setupWindSensor() {
	std::string val;
	try {
		m_windSensor.setController(&m_maestroController);
		val = m_dbHandler.retriveCell("configs", "1", "ws_chan");
		m_windSensor.setChannel(atoi(val.c_str()));
	} catch (const char * error) {
		logError(error);
		exit(1);
	}
}

void SailingRobot::setupGPS() {
	try {
		m_gpsReader.connectToGPS();
	} catch (const char * error) {
		logError(error);
		exit(1);
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
		logError(error);
		exit(1);
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
		logError(error);
		exit(1);
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
		logError(error);
		exit(1);
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
		logError(error);
		exit(1);
	}*/
}


void SailingRobot::setupHTTPSync() {
	std::string val;
	try {
		val = m_dbHandler.retriveCell("server", "1", "boat_id");
		m_httpSync.setShipID(val);
		val = m_dbHandler.retriveCell("server", "1", "boat_pwd");
		m_httpSync.setShipPWD(string shipPWD);
		val = m_dbHandler.retriveCell("server", "1", "srv_addr");
		m_httpSync.setServerURL(string URL);
	} catch (const char * error) {
		logError(error);
		exit(1);
	}
}

void SailingRobot::syncServer() {
	JSONData data1;
	data1.add("id", m_dbHandler.retriveCell("datalogs", "1", "id"));
	data1.add("gps_time",m_dbHandler.retriveCell("datalogs", "1", "gps_time"));
	data1.add("gps_lat",m_dbHandler.retriveCell("datalogs", "1", "gps_lat"));
	data1.add("gps_lon",m_dbHandler.retriveCell("datalogs", "1", "gps_lon"));
	data1.add("gps_spd",m_dbHandler.retriveCell("datalogs", "1", "gps_spd"));
	data1.add("gps_head",m_dbHandler.retriveCell("datalogs", "1", "gps_head"));
	data1.add("gps_sat",m_dbHandler.retriveCell("datalogs", "1", "gps_sat"));
	data1.add("sc_cmd",m_dbHandler.retriveCell("datalogs", "1", "sc_cmd"));
	data1.add("rc_cmd",m_dbHandler.retriveCell("datalogs", "1", "rc_cmd"));
	data1.add("ss_pos",m_dbHandler.retriveCell("datalogs", "1", "ss_pos"));
	data1.add("rs_pos",m_dbHandler.retriveCell("datalogs", "1", "rs_pos"));
	data1.add("cc_dtw",m_dbHandler.retriveCell("datalogs", "1", "cc_dtw"));
	data1.add("cc_btw",m_dbHandler.retriveCell("datalogs", "1", "cc_btw"));
	data1.add("cc_cts",m_dbHandler.retriveCell("datalogs", "1", "cc_cts"));
	data1.add("cc_tack",m_dbHandler.retriveCell("datalogs", "1", "cc_tack"));
	data1.add("ws_dir",m_dbHandler.retriveCell("datalogs", "1", "ws_dir"));
	data1.add("ws_spd",m_dbHandler.retriveCell("datalogs", "1", "ws_spd"));
	data1.add("ws_tmp",m_dbHandler.retriveCell("datalogs", "1", "ws_tmp"));
	data1.add("cfg_rev","cfg0001");
	data1.add("rte_rev","rte0001");
	data1.add("wpt_cur",m_dbHandler.retriveCell("datalogs", "1", "wpt_cur"));
	data1.add("wpt_rev","wpt0001");

	JSONData data2;
	data2.add("id",m_dbHandler.retriveCell("messages", "1", "id"));
	data2.add("gps_time",m_dbHandler.retriveCell("messages", "1", "gps_time"));
	data2.add("type",m_dbHandler.retriveCell("messages", "1", "type"));
	data2.add("msg",m_dbHandler.retriveCell("messages", "1", "msg"));
	data2.add("log_id",m_dbHandler.retriveCell("messages", "1", "log_id"));

	JSONBlock blck1; blck1.add(&data1);
	JSONBlock blck2; blck2.add(&data2);

	JSONArray logs;
	logs.setName("datalogs");
	logs.add(blck1);

	JSONArray messages;
	messages.setName("messages");
	messages.add(blck2);

	JSONBlock main;
	main.add(&logs);
	main.add(&messages);

	m_httpSync.pushLogs(main.toString());
}

