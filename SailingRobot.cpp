#include "SailingRobot.h"
#include <cstdlib>
#include <iostream>
#include <wiringPi.h>
#include <unistd.h>
#include <fstream>
#include <cstring>
#include <cmath>

SailingRobot::SailingRobot(ExternalCommand* externalCommand,
						   SystemState *systemState, DBHandler *db) :
	m_mockWindsensor(true),
	m_mockCompass(true),
	m_mockPosition(true),
	m_mockMaestro(true),

	m_dbHandler(db),

	m_externalCommand(externalCommand),
	m_systemState(systemState)
{
/*	sleepypi stuff
	wiringPiSetup();
	pinMode(6, OUTPUT);
	digitalWrite(6, HIGH);*/
}

SailingRobot::~SailingRobot() {
/*	sleepypi stuff
	digitalWrite(6, LOW);gtk
	std::cout << "sr destruct\n";*/
}


void SailingRobot::init(std::string programPath, std::string errorFileName) {
	m_errorLogPath = programPath + errorFileName;

	m_getHeadingFromCompass = m_dbHandler->retriveCellAsInt("configs", "1", "flag_heading_compass");

	printf(" Starting HTTPSync\t\t");
	setupHTTPSync();
	printf("OK\n");

	printf(" Starting Compass\t\t");
	setupCompass();
	printf("OK\n");

	printf(" Starting Windsensor\t\t");
	setupWindSensor();
	printf("OK\n");

	printf(" Starting Maestro\t\t");
	setupMaestro();
	printf("OK\n");

	printf(" Starting RudderServo\t\t");
	setupRudderServo();
	printf("OK\n");

	printf(" Starting RudderCommand\t\t");
	setupRudderCommand();
	printf("OK\n");

	printf(" Starting SailServo\t\t");
	setupSailServo();
	printf("OK\n");

	printf(" Starting SailCommand\t\t");
	setupSailCommand();
	printf("OK\n");

	printf(" Starting Waypoint\t\t");
	setupWaypoint();
	printf("OK\n");

	printf(" Starting CourseCalculation\t");
	setupCourseCalculation();
	printf("OK\n");

	//updateState();
	//syncServer();
}

int SailingRobot::getHeading(){

	int newHeading = 0;

	if (m_getHeadingFromCompass) {
			newHeading = m_compass->getHeading();
		}
		else {
			newHeading = m_gpsReader->getHeading();
		}

	return newHeading;

}

int SailingRobot::mockLatitude(int oldLat){

	double courseToSteer = m_courseCalc.getCTS();

	if(courseToSteer > 90 && courseToSteer < 270 && courseToSteer != 0){

			oldLat -= 0.1;
	}else if (courseToSteer != 0){
			oldLat += 0.1;
	}

	return oldLat;

}

int SailingRobot::mockLongitude(int oldLong){

	double courseToSteer = m_courseCalc.getCTS();

	if (courseToSteer < 180 && courseToSteer != 0){

			oldLong += 0.1;

	}else if (courseToSteer != 0){
					oldLong -= 0.1;
	}

	return oldLong;

}

void SailingRobot::run() {
	sleep(3);
	m_running=true;
	int rudderCommand, sailCommand, windDir, twd, heading = 0;
	double longitude = 4, latitude = -3;
	printf("*SailingRobot::run() started.\n");
	std::cout << "waypoint target." << std::endl 
		<< "long: " << m_waypointLongitude << std::endl
		<< "lat : " << m_waypointLatitude << std::endl;
	while(m_running) {
		//m_waypointId.empty()

		try {
			m_windSensor->parseData(m_windSensor->refreshData());	
		} catch(const char * e) {
			std::cout << "ERROR: SailingRobot::Run m_windSensor->parseData " << e << std::endl;
		}
		

		windDir = m_windSensor->getDirection();


		


		m_compass->readValues();
		heading = getHeading();
		


		if ( !isnan(m_gpsReader->getLatitude()) ) {

			//calc DTW
			if (m_mockPosition) {


				longitude = mockLongitude(longitude);
				latitude = mockLatitude(latitude);
				
				double courseToSteer = m_courseCalc.getCTS();


				if (heading > courseToSteer){

					heading--;

				}else if (heading < courseToSteer){

					heading++;
				}else heading = courseToSteer;

			}
			else {
				longitude = m_gpsReader->getLongitude();
				latitude = m_gpsReader->getLatitude();
			}

			m_courseCalc.calculateDTW(latitude, longitude,
				m_waypointLatitude, m_waypointLongitude);

			//calc & set TWD
			twd = heading + windDir;

			if (twd > 359) {
				twd -= 360;
			}
			if (twd < 0) {
				twd += 360;
			}
			m_courseCalc.setTWD(twd);

			//calc BTW & CTS
			m_courseCalc.calculateBTW(latitude, longitude,
				m_waypointLatitude, m_waypointLongitude);
			m_courseCalc.calculateCTS();

			//rudder position calculation
			rudderCommand = m_rudderCommand.getCommand(m_courseCalc.getCTS(), heading);
			if(!m_externalCommand->getAutorun()){
				rudderCommand = m_externalCommand->getRudderCommand();
			}

		} else {
			logMessage("error", "SailingRobot::run(), gps NaN. Using values from last iteration.");
		}

		//sail position calculation
		sailCommand = m_sailCommand.getCommand(windDir);
		if(!m_externalCommand->getAutorun()){
			sailCommand = m_externalCommand->getSailCommand();
		}

		//rudder adjustment
		m_rudderServo.setPosition(rudderCommand);
		//sail adjustment
		m_sailServo.setPosition(sailCommand);

		//update system state
		m_systemState->setWindsensorModel(WindsensorModel(
				m_windSensor->getDirection(),
				m_windSensor->getSpeed(),
				m_windSensor->getTemperature()
			));
		m_systemState->setCompassModel(CompassModel(
				m_compass->getHeading(),
				m_compass->getPitch(),
				m_compass->getRoll()
			));
		m_systemState->setRudder(rudderCommand);
		m_systemState->setSail(sailCommand);


		//logging
		m_dbHandler->insertDataLog(
			m_gpsReader->getTimestamp(),
			latitude,
			longitude,
			m_gpsReader->getSpeed(),
			m_gpsReader->getHeading(),
			m_gpsReader->getSatellitesUsed(),
			sailCommand,
			rudderCommand,
			0, //sailservo getpos, to remove
			0, //rudderservo getpos, to remove
			m_courseCalc.getDTW(),
			m_courseCalc.getBTW(),
			m_courseCalc.getCTS(),
			m_courseCalc.getTACK(),
			windDir,
			m_windSensor->getSpeed(),
			m_windSensor->getTemperature(),
			atoi(m_waypointId.c_str()),
			m_compass->getHeading(),
			m_compass->getPitch(),
			m_compass->getRoll()
		);

//		syncServer();

		//check if we are within 15meters of the waypoint and move to next wp in that case
		if (m_courseCalc.getDTW() < 15) {

			//remove this cout later
			
			nextWaypoint();
			setupWaypoint();
		}
	}
	printf("*SailingRobot::run() exiting\n");
}


void SailingRobot::shutdown() {
//	syncServer();
	m_running=false;
	m_dbHandler->closeDatabase();
}


void SailingRobot::logMessage(std::string type, std::string message) {
	try {
		m_dbHandler->insertMessageLog(m_gpsReader->getTimestamp(), type, message);
       	} catch (const char * logError) {
		std::ofstream errorFile;
		errorFile.open(m_errorLogPath.c_str(), std::ios::app);
		errorFile << "log error: " << logError << "\n";
		errorFile << "when logging " << type << ": " << message << "\n";
		errorFile.close();
	}
}

void SailingRobot::syncServer() {
	try {
		std::string response = m_httpSync.pushLogs( m_dbHandler->getLogs() );
		m_dbHandler->removeLogs(response);
	} catch (const char * error) {
		logMessage("error", error);
	}
}


void SailingRobot::updateState() {
	try {
		std::string setup = m_httpSync.getSetup();
		bool stateChanged = false;
		if (m_dbHandler->revChanged("cfg_rev", setup) ) {
			m_dbHandler->updateTable("configs", m_httpSync.getConfig());
			stateChanged = true;
			logMessage("message", "config state updated");
		}
		if (m_dbHandler->revChanged("rte_rev", setup) ) {
			m_dbHandler->updateTable("waypoints", m_httpSync.getRoute());
			stateChanged = true;
			logMessage("message", "route state updated");
		}
		if (stateChanged)  {
			m_dbHandler->updateTable("state", m_httpSync.getSetup());
		}
	} catch (const char * error) {
		logMessage("error", error);
	}
}


void SailingRobot::nextWaypoint() {

	try {
		m_dbHandler->deleteRow("waypoints", m_waypointId);
	} catch (const char * error) {
		logMessage("error", error);
	}
	logMessage("message", "SailingRobot::nextWaypoint(), waypoint reached");
	std::cout << "Waypoint reached!" << std::endl;
}

void SailingRobot::setupMaestro() {
	if (m_mockMaestro) {
		m_maestroController.reset(new MockMaestroController());
	}
	else {
		m_maestroController.reset(new MaestroController());
	}

	std::string port_name;
	try {
		port_name = m_dbHandler->retriveCell("configs", "1", "mc_port");
	} catch (const char * error) {
		logMessage("error", error);
	}

	try {
		m_maestroController->setPort( port_name );
		int maestroErrorCode = m_maestroController->getError();
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
		m_rudderServo.setController(m_maestroController.get());
		m_rudderServo.setChannel( m_dbHandler->retriveCellAsInt("configs", "1", "rs_chan") );
		m_rudderServo.setSpeed( m_dbHandler->retriveCellAsInt("configs", "1", "rs_spd") );
		m_rudderServo.setAcceleration( m_dbHandler->retriveCellAsInt("configs", "1", "rs_acc") );
	} catch (const char * error) {
		logMessage("error", error);
		throw;
	}
	logMessage("message", "setupRudderServo() done");
}

void SailingRobot::setupSailServo() {
	try {
		m_sailServo.setController(m_maestroController.get());
		m_sailServo.setChannel( m_dbHandler->retriveCellAsInt("configs", "1", "ss_chan") );
		m_sailServo.setSpeed( m_dbHandler->retriveCellAsInt("configs", "1", "ss_spd") );
		m_sailServo.setAcceleration( m_dbHandler->retriveCellAsInt("configs", "1", "ss_acc") );
	} catch (const char * error) {
		logMessage("error", error);
		throw;
	}
	logMessage("message", "setupSailServo() done");
}

void SailingRobot::setupWindSensor() {
	std::string port_name = "non";
	int baud_rate = 1;
	int buff_size = 1;

	try {
		port_name = m_dbHandler->retriveCell("configs", "1", "ws_port");
		baud_rate = m_dbHandler->retriveCellAsInt("configs", "1", "ws_baud");
		buff_size = m_dbHandler->retriveCellAsInt("configs", "1", "ws_buff");
	} catch (const char * error) {
		logMessage("error", error);
	}
	if (!m_mockWindsensor) {
		m_windSensor = new CV7;
	}
	else {
		m_windSensor = new MockWindsensor;
	}

	try {
		m_windSensor->loadConfig( port_name, baud_rate );
		m_windSensor->setBufferSize( buff_size );
	} catch (const char * error) {
		logMessage("error", error);
		throw error;
	}
	logMessage("message", "setupWindSensor() done");
}

void SailingRobot::setupCourseCalculation() {
	try {
		m_courseCalc.setTACK_ANGLE( m_dbHandler->retriveCellAsInt("configs", "1", "cc_ang_tack") );
		m_courseCalc.setSECTOR_ANGLE( m_dbHandler->retriveCellAsInt("configs", "1", "cc_ang_sect") );
	} catch (const char * error) {
		logMessage("error", error);
		throw;
	}
	logMessage("message", "setupCourseCalculation() done");
}

void SailingRobot::setupRudderCommand() {
	try {
		m_rudderCommand.setCommandValues( m_dbHandler->retriveCellAsInt("configs", "1", "rc_cmd_xtrm"),
			m_dbHandler->retriveCellAsInt("configs", "1", "rc_cmd_mid"));

	} catch (const char * error) {
		logMessage("error", error);
		throw;
	}
	logMessage("message", "setupRudderCommand() done");
}

void SailingRobot::setupSailCommand() {
	try {
		m_sailCommand.setCommandValues( m_dbHandler->retriveCellAsInt("configs", "1", "sc_cmd_clse"),
			m_dbHandler->retriveCellAsInt("configs", "1", "sc_cmd_run"));

	} catch (const char * error) {
		logMessage("error", error);
		throw;
	}
	logMessage("message", "setupSailCommand() done");
}

void SailingRobot::setupWaypoint() {
	std::string id;
	std::string lat;
	std::string lon;
	try {
		id = m_dbHandler->getMinIdFromTable("waypoints");
		lat = m_dbHandler->retriveCell("waypoints", id, "lat");
		lon = m_dbHandler->retriveCell("waypoints", id, "lon");
	} catch (const char * error) {
		logMessage("error", error);
	}

	try {
		m_waypointId = id;
		if (m_waypointId.empty()) {
			throw "No waypoint found!";
		}
		m_waypointLatitude = atof(lat.c_str());
		m_waypointLongitude = atof(lon.c_str());

		std::cout << "New waypoint picked!" << m_waypointLatitude << m_waypointLongitude << std::endl;
	} catch (const char * error) {
		logMessage("error", error);
		throw;
	}
	logMessage("message", "setupWaypoint() done");

	
}

void SailingRobot::setupHTTPSync() {
	try {
		m_httpSync.setShipID( m_dbHandler->retriveCell("server", "1", "boat_id") );
		m_httpSync.setShipPWD( m_dbHandler->retriveCell("server", "1", "boat_pwd") );
		m_httpSync.setServerURL( m_dbHandler->retriveCell("server", "1", "srv_addr") );
	} catch (const char * error) {
		logMessage("error", "SailingRobot::setupHTTPSync() failed");
	}
	logMessage("message", "setupHTTPSync() done");
}

void SailingRobot::setupCompass() {
	if (!m_mockCompass) {
		m_compass = new HMC6343;
	}
	else {
		m_compass = new MockCompass;
	}
	try {
		m_compass->init();
	} catch (const char * error) {
		logMessage("error", "SailingRobot::setupCompass() failed");
	}
	logMessage("message", "setupCompass() done");
}
