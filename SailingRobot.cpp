#include "SailingRobot.h"
#include <cstdlib>
#include <iostream>
#include <unistd.h>
#include <fstream>
#include <cstring>
#include <cmath>
#include "utility/Utility.h"
#include "utility/Timer.h"
#include "servocontroller/MockMaestroController.h"
#include "Compass/HMC6343.h"
#include "Compass/MockCompass.h"
#include "windvanecontroller/WindVaneController.h"


SailingRobot::SailingRobot(ExternalCommand* externalCommand,
						   SystemState *systemState, DBHandler *db) :

	m_mockCompass(db->retriveCellAsInt("mock", "1", "Compass")),
	m_mockPosition(db->retriveCellAsInt("mock", "1", "Position")),
	m_mockMaestro(db->retriveCellAsInt("mock", "1", "Maestro")),

	m_dbHandler(db),

	m_waypointModel(PositionModel(1.5,2.7), 100, "", 6),
	m_waypointRouting(m_waypointModel,
		atof(m_dbHandler->retriveCell("configs", "1", "wp_inner_radius_ratio").c_str()),
		m_dbHandler->retriveCellAsInt("configs", "1", "cc_tack_ang"),
		m_dbHandler->retriveCellAsInt("configs", "1", "cc_tack_max_ang"),
		atof(m_dbHandler->retriveCell("configs", "1", "cc_tack_min_spd").c_str()),
		m_dbHandler->retriveCellAsInt("configs", "1", "cc_ang_sect")),

	m_externalCommand(externalCommand),
	m_systemState(systemState),

	m_systemStateModel(
		SystemStateModel(
			GPSModel("",PositionModel(0,0),0,0,0,0),
			WindsensorModel(0,0,0),
			CompassModel(0,0,0),
			0,
			0
		)
	)
{

}

SailingRobot::~SailingRobot() {

}


void SailingRobot::init(std::string programPath, std::string errorFileName) {
	m_errorLogPath = programPath + errorFileName;

	m_getHeadingFromCompass = m_dbHandler->retriveCellAsInt("configs", "1", "flag_heading_compass");
	
	/* 
	printf(" Starting HTTPSync\t\t");
	setupHTTPSync();
	printf("OK\n");
	*/

	printf(" Starting Compass\t\t");
	setupCompass();
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

	m_waypointRouting.setWaypoint(m_waypointModel);

	//updateState();
	//syncServer();
}

int SailingRobot::getHeading(int declination) {

	int newHeading = 0;

	if (m_getHeadingFromCompass) {
		newHeading = Utility::addDeclinationToHeading(m_compass->getHeading(), m_waypointModel.declination);
	}
	else {
		newHeading = m_gpsReader->getHeading();
	}

	return newHeading;
}

double SailingRobot::mockLatitude(double oldLat, double cts) {
	oldLat += cos(Utility::degreeToRadian(cts)) * 0.0002;
	return oldLat;
}

double SailingRobot::mockLongitude(double oldLong, double cts) {
	oldLong += sin(Utility::degreeToRadian(cts)) * 0.0002;
	return oldLong;
}

void SailingRobot::run() {

	m_running = true;
	int rudderCommand, sailCommand, windDir, heading = 0, insertScanOnce = 0;
	std::vector<float> twdBuffer;
	const unsigned int twdBufferMaxSize =
		m_dbHandler->retriveCellAsInt("buffer_configs", "1", "true_wind");
	
	//double longitude = 4, latitude = -3;
	double latitude = 60.098933, longitude = 19.921028;
	
	Timer timer;
	std::string sr_loop_time =
		m_dbHandler->retriveCell("configs", "1", "sr_loop_time");
	double loop_time = std::stod(sr_loop_time);

	printf("*SailingRobot::run() started.\n");
	std::cout << "Waypoint target - ID: " << m_waypointModel.id << " lon: " << 
	m_waypointModel.positionModel.longitude	<< " lat : " << 
	m_waypointModel.positionModel.latitude << std::endl;

	while(m_running) {
		timer.reset();

		//Get data from SystemStateModel to local object
		m_systemState->getData(m_systemStateModel);
		
		windDir = m_systemStateModel.windsensorModel.direction;

		m_compass->readValues();
		heading = getHeading(m_waypointModel.declination);
		
		std::cout << "heading: " << heading << "\n";
		std::cout << "headeing ssm compass:" << m_systemStateModel.compassModel.heading<<"\n";
		
		if (m_systemStateModel.gpsModel.online) {

			//calc DTW
			if (m_mockPosition) {

				longitude = mockLongitude(longitude, m_waypointRouting.getCTS());
				latitude = mockLatitude(latitude, m_waypointRouting.getCTS());

				if (heading > m_waypointRouting.getCTS()) {

					heading--;
				} else if (heading < m_waypointRouting.getCTS()) {

					heading++;
				} else heading = m_waypointRouting.getCTS();

			} else {
				longitude = m_systemStateModel.gpsModel.positionModel.longitude;
				latitude = m_systemStateModel.gpsModel.positionModel.latitude;
			}

			//calc & set TWD
			twdBuffer.push_back(heading + windDir);
			while (twdBuffer.size() > twdBufferMaxSize) {
				twdBuffer.erase(twdBuffer.begin());
			}

			double rudder = 0, sail = 0;
			m_waypointRouting.getCommands(rudder, sail,
				PositionModel(latitude, longitude),
				Utility::meanOfAngles(twdBuffer), heading, m_systemStateModel);
				

			if(m_dbHandler->retriveCellAsInt("configs", "1", "use_self_steering")) {
				if(m_dbHandler->retriveCellAsInt("configs", "1", "wind_sensor_self_steering")) {
					m_windVaneController.setVaneAngle(m_waypointRouting.getTWD(), m_waypointRouting.getCTS());
					// WindVaneController::getAngle() will fetch set angle of vane.
					// To do: Pass angle to engine controlling wind vane.
				} else {
					// For steering without wind sensor.
				}
			}

			rudderCommand = m_rudderCommand.getCommand(rudder);
			if(!m_externalCommand->getAutorun()) {
				rudderCommand = m_externalCommand->getRudderCommand();
			}
			sailCommand = m_sailCommand.getCommand(sail);
			if(!m_externalCommand->getAutorun()) {
				sailCommand = m_externalCommand->getSailCommand();
			}

		} else {
			m_logger.error("SailingRobot::run(), gps NaN. Using values from last iteration.");
		}

		//rudder adjustment
		m_rudderServo.setPosition(rudderCommand);
		//sail adjustment
		m_sailServo.setPosition(sailCommand);

		//update system state
		m_systemState->setCompassModel(CompassModel(
				m_compass->getHeading(),
				m_compass->getPitch(),
				m_compass->getRoll()
			));

		std::cout << "headeing ssm compass after setmodel:" << m_systemStateModel.compassModel.heading<<"\n";

		m_systemState->setRudder(rudderCommand);
		m_systemState->setSail(sailCommand);


		//logging
		m_dbHandler->insertDataLog(
			m_systemStateModel,
			0, //sailservo getpos, to remove
			0, //rudderservo getpos, to remove
			m_waypointRouting.getDTW(),
			m_waypointRouting.getBTW(),
			m_waypointRouting.getCTS(),
			m_waypointRouting.getTack(),
			m_waypointRouting.getGoingStarboard(),
			atoi(m_waypointModel.id.c_str()),
			Utility::meanOfAngles(twdBuffer)
		);

//		syncServer();

		// check if we are within the radius of the waypoint
		// and move to next wp in that case
		if (m_waypointRouting.nextWaypoint(PositionModel(latitude, longitude))) {
				
			// check if m_waypointModel.id exists in waypoint_index
			int i = m_dbHandler->retriveCellAsInt("waypoint_index", m_waypointModel.id, "id");
			if (m_dbHandler->retriveCellAsInt("configs", "1", "scanning") && i != 0 && insertScanOnce != i)
			{
				insertScanOnce = i;				
				try {
					m_dbHandler->insertScan(m_waypointModel.id, PositionModel(latitude,longitude),
						m_systemStateModel.windsensorModel.temperature,
						m_systemStateModel.gpsModel.utc_timestamp);
				} catch (const char * error) {
					m_logger.error(error);
					std::cout << error << std::endl;
				}
			}

			nextWaypoint();
			setupWaypoint();
			m_waypointRouting.setWaypoint(m_waypointModel);
		
		}
	
			

		timer.sleepUntil(loop_time);
	}

	printf("*SailingRobot::run() exiting\n");
}

void SailingRobot::shutdown() {
//	syncServer();
	m_running=false;
	m_dbHandler->closeDatabase();
}



void SailingRobot::nextWaypoint() {

	try {
		m_dbHandler->changeOneValue("waypoints", m_waypointModel.id,"1","harvested");
	} catch (const char * error) {
		m_logger.error(error);
	}
	m_logger.info("SailingRobot::nextWaypoint(), waypoint reached");
	std::cout << "Waypoint reached!" << std::endl;
	
}

void SailingRobot::setupWaypoint() {

	try {
		m_dbHandler->getWaypointFromTable(m_waypointModel);		
	} catch (const char * error) {
		m_logger.error(error);
	}
	try {
		if (m_waypointModel.id.empty() ) {
			std::cout << "No waypoint found!"<< std::endl;			
			throw "No waypoint found!";			
		}
		else{
			std::cout << "New waypoint picked! ID:" << m_waypointModel.id <<" lat: "
			<< m_waypointModel.positionModel.latitude 
			<< " lon: " << m_waypointModel.positionModel.longitude << " rad: " 
			<< m_waypointModel.radius << std::endl;
		}
	} catch (const char * error) {
		m_logger.error(error);		
		//m_dbHandler->insertMessageLog("NOTIME", "NOTYPE", "NO WAYPOINT FOUND!");
		//throw;
	}

	m_logger.info("setupWaypoint() done");
}

void SailingRobot::setupMaestro() {
	if (m_mockMaestro) {
		m_maestroController.reset(new MockMaestroController());
	} else {
		m_maestroController.reset(new MaestroController());
	}

	std::string port_name;
	try {
		port_name = m_dbHandler->retriveCell("configs", "1", "mc_port");
	} catch (const char * error) {
		m_logger.error(error);
	}

	try {
		m_maestroController->setPort( port_name );
		int maestroErrorCode = m_maestroController->getError();
		if (maestroErrorCode != 0) {
			std::stringstream stream;
			stream << "maestro errorcode: " << maestroErrorCode;
			m_logger.error(stream.str());
		}
	} catch (const char * error) {
		m_logger.error(error);
		throw;
	}
	m_logger.info("setupMaestro() done");
}

void SailingRobot::setupRudderServo() {
	try {
		m_rudderServo.setController(m_maestroController.get());
		m_rudderServo.setChannel( m_dbHandler->retriveCellAsInt("configs", "1", "rs_chan") );
		m_rudderServo.setSpeed( m_dbHandler->retriveCellAsInt("configs", "1", "rs_spd") );
		m_rudderServo.setAcceleration( m_dbHandler->retriveCellAsInt("configs", "1", "rs_acc") );
	} catch (const char * error) {
		m_logger.error(error);
		throw;
	}
	m_logger.info("setupRudderServo() done");
}

void SailingRobot::setupSailServo() {
	try {
		m_sailServo.setController(m_maestroController.get());
		m_sailServo.setChannel( m_dbHandler->retriveCellAsInt("configs", "1", "ss_chan") );
		m_sailServo.setSpeed( m_dbHandler->retriveCellAsInt("configs", "1", "ss_spd") );
		m_sailServo.setAcceleration( m_dbHandler->retriveCellAsInt("configs", "1", "ss_acc") );
	} catch (const char * error) {
		m_logger.error(error);
		throw;
	}
	m_logger.info("setupSailServo() done");
}

void SailingRobot::setupRudderCommand() {
	try {
		m_rudderCommand.setCommandValues( m_dbHandler->retriveCellAsInt("configs", "1", "rc_cmd_xtrm"),
			m_dbHandler->retriveCellAsInt("configs", "1", "rc_cmd_mid"));

	} catch (const char * error) {
		m_logger.error(error);
		throw;
	}
	m_logger.info("setupRudderCommand() done");
}

void SailingRobot::setupSailCommand() {
	try {
		m_sailCommand.setCommandValues( m_dbHandler->retriveCellAsInt("configs", "1", "sc_cmd_clse"),
			m_dbHandler->retriveCellAsInt("configs", "1", "sc_cmd_run"));

	} catch (const char * error) {
		m_logger.error(error);
		throw;
	}
	m_logger.info("setupSailCommand() done");
}




void SailingRobot::setupCompass() {
	if (!m_mockCompass) {
		m_compass = new HMC6343(
			m_dbHandler->retriveCellAsInt("buffer_configs", "1", "compass") );
	} else {
		m_compass = new MockCompass;
	}
	try {
		m_compass->init();
	} catch (const char * error) {
		m_logger.error("SailingRobot::setupCompass() failed");
	}
	m_logger.info("setupCompass() done");
}
