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
#include "behaviourclass/WaypointBehaviour.h"
#include "behaviourclass/LineFollowBehaviour.h"



SailingRobot::SailingRobot(ExternalCommand* externalCommand,
						   SystemState *systemState, DBHandler *db) :
	m_mockPosition(db->retrieveCellAsInt("mock", "1", "position")),
	m_mockMaestro(db->retrieveCellAsInt("mock", "1", "maestro")),

	m_dbHandler(db),

	m_externalCommand(externalCommand),
	m_systemState(systemState),

	m_systemStateModel(
		SystemStateModel(
			GPSModel("",PositionModel(0,0),0,0,0,0),
			WindsensorModel(0,0,0),
			CompassModel(0,0,0,AccelerationModel(0,0,0) ),
			AnalogArduinoModel(0, 0, 0, 0),
			0,
			0
		)
	)
{
        if(m_mockPosition) { position.reset(new MockPosition() );  }
        else { position.reset(new RealPosition(m_systemStateModel) );  }
			
}

SailingRobot::~SailingRobot() {

}


void SailingRobot::init(std::string programPath, std::string errorFileName) {
	m_errorLogPath = programPath + errorFileName;

	m_getHeadingFromCompass = m_dbHandler->retrieveCellAsInt("sailing_robot_config", "1", "flag_heading_compass");

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
}

void SailingRobot::run() {

	//m_dbHandler->clearLogs();
	m_running = true;
	routeStarted = true;
	int rudderCommand, sailCommand;//,heading = 0, insertScanOnce = 0;
	//int windDir = 0; // outComment if use of tureWindDirCalculation
	std::vector<float> twdBuffer;
	const unsigned int twdBufferMaxSize =
		m_dbHandler->retrieveCellAsInt("buffer_config", "1", "true_wind");
	double trueWindDirection;

	Timer timer;
	std::string sr_loop_time =
		m_dbHandler->retrieveCell("sailing_robot_config", "1", "loop_time");
	double loop_time = std::stod(sr_loop_time);

	bool usingLineFollow = std::stoi(m_dbHandler->retrieveCell("sailing_robot_config", "1", "line_follow"));
	//bool previousBehaviour = usingLineFollow; //Used in while-loop to see if waypoint routing has changed


  	WaypointBehaviour waypB(m_dbHandler); 
  	LineFollowBehaviour LineFollowB(m_dbHandler);
	RoutingBehaviour *behave;
	if(usingLineFollow)  //Get which system to use
		behave = &LineFollowB;
	else
		behave = &waypB;
  	behave->init();
	while(m_running) {
		timer.reset();

		// usingLineFollow = std::stoi(m_dbHandler->retrieveCell("sailing_robot_config", "1", "line_follow"));
		// if (usingLineFollow != previousBehaviour){ //If following behaviour changes in database
		// 	if(previousBehaviour)
		// 		behave = &waypB;						
		// 	else									//AN OPTION TO CHANGE ROUTING BEHAVIOUR DURING RUN. NOT SMART TO CHECK DBHandler EVERY TIME??? /Oliver
		// 		behave = &LineFollowB;

		// 	behave->init();
		// 	previousBehaviour = usingLineFollow;
		// }
		
		//Get data from SystemStateModel to local object
		m_systemState->getData(m_systemStateModel);

		//calc & set TWD
		trueWindDirection = Utility::getTrueWindDirection(m_systemStateModel, twdBuffer, twdBufferMaxSize);
		
		//Compute the commands to send
		behave->computeCommands(m_systemStateModel,position, trueWindDirection ,m_mockPosition, m_getHeadingFromCompass);


		rudderCommand = m_rudderCommand.getCommand(behave->getRudderCommand());
		if(!m_externalCommand->getAutorun()) {
			rudderCommand = m_externalCommand->getRudderCommand();
		}
		sailCommand = m_sailCommand.getCommand(behave->getSailCommand());
		if(!m_externalCommand->getAutorun()) {
			sailCommand = m_externalCommand->getSailCommand();
		}

		printf("sailCommand: "); printf(std::to_string(sailCommand).c_str()); printf("    : rudderCommand: "); printf(std::to_string(rudderCommand).c_str()); printf("\n");

		//rudder adjustment
		m_rudderServo.setPosition(rudderCommand);
		//sail adjustment
		m_sailServo.setPosition(sailCommand);

		m_systemState->setRudder(rudderCommand);
		m_systemState->setSail(sailCommand);

		//Save data in database
		behave->manageDatabase(twdBuffer,m_systemStateModel);

		timer.sleepUntil(loop_time);
	}

	printf("*SailingRobot::run() exiting\n");
}

void SailingRobot::shutdown() {
	m_running=false;
	//m_dbHandler->closeDatabase();
}


void SailingRobot::setupMaestro() {
	if (m_mockMaestro) {
		m_maestroController.reset(new MockMaestroController());
	} else {
		m_maestroController.reset(new MaestroController());
	}

	std::string port_name;
	try {
		port_name = m_dbHandler->retrieveCell("maestro_controller_config", "1", "port");
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
		m_rudderServo.setChannel( m_dbHandler->retrieveCellAsInt("rudder_servo_config", "1", "channel") );
		m_rudderServo.setSpeed( m_dbHandler->retrieveCellAsInt("rudder_servo_config", "1", "speed") );
		m_rudderServo.setAcceleration( m_dbHandler->retrieveCellAsInt("rudder_servo_config", "1", "acceleration") );
	} catch (const char * error) {
		m_logger.error(error);
		throw;
	}
	m_logger.info("setupRudderServo() done");
}

void SailingRobot::setupSailServo() {
	try {
		m_sailServo.setController(m_maestroController.get());
		m_sailServo.setChannel( m_dbHandler->retrieveCellAsInt("sail_servo_config", "1", "channel") );
		m_sailServo.setSpeed( m_dbHandler->retrieveCellAsInt("sail_servo_config", "1", "speed") );
		m_sailServo.setAcceleration( m_dbHandler->retrieveCellAsInt("sail_servo_config", "1", "acceleration") );
	} catch (const char * error) {
		m_logger.error(error);
		throw;
	}
	m_logger.info("setupSailServo() done");
}

void SailingRobot::setupRudderCommand() {
	try {
		m_rudderCommand.setCommandValues( m_dbHandler->retrieveCellAsInt("rudder_command_config", "1", "extreme_command"),
			m_dbHandler->retrieveCellAsInt("rudder_command_config", "1", "midship_command"));

	} catch (const char * error) {
		m_logger.error(error);
		throw;
	}
	m_logger.info("setupRudderCommand() done");
}

void SailingRobot::setupSailCommand() {
	try {
		m_sailCommand.setCommandValues( m_dbHandler->retrieveCellAsInt("sail_command_config", "1", "close_reach_command"),
			m_dbHandler->retrieveCellAsInt("sail_command_config", "1", "run_command"));

	} catch (const char * error) {
		m_logger.error(error);
		throw;
	}
	m_logger.info("setupSailCommand() done");
}