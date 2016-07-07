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
						   SystemState *systemState, DBHandler *db, HTTPSync* httpSync) :
	m_mockPosition(db->retrieveCellAsInt("mock", "1", "position")),
	m_mockMaestro(db->retrieveCellAsInt("mock", "1", "maestro")),

	

	
	m_externalCommand(externalCommand),
	m_systemState(systemState),
	m_dbHandler(db),
	m_httpSync(httpSync),
	m_waypointModel(PositionModel(1.5,2.7), 100, "", 6),
	m_waypointRouting(m_waypointModel,
		atof(m_dbHandler->retrieveCell("waypoint_routing_config", "1", "radius_ratio").c_str()),
		atof(m_dbHandler->retrieveCell("course_calculation_config", "1", "tack_angle").c_str()),
		atof(m_dbHandler->retrieveCell("course_calculation_config", "1", "tack_max_angle").c_str()),
		atof(m_dbHandler->retrieveCell("course_calculation_config", "1", "tack_min_speed").c_str()),
		atof(m_dbHandler->retrieveCell("course_calculation_config", "1", "sector_angle").c_str()),
		 atof(m_dbHandler->retrieveCell("waypoint_routing_config", "1", "max_command_angle ").c_str()),
		 atof(m_dbHandler->retrieveCell("waypoint_routing_config", "1", "rudder_speed_min").c_str())
		),


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

	if (m_getHeadingFromCompass){
		m_gpsHeadingWeight = 0.0;
	}else m_gpsHeadingWeight = 1.0;

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

int SailingRobot::getHeading() {

	//Use GPS for heading only if speed is higher than 1 knot
	int useGpsForHeadingKnotSpeed = 1;
	bool gpsForbidden = Utility::directionAdjustedSpeed(m_systemStateModel.gpsModel.heading, m_systemStateModel.compassModel.heading, m_systemStateModel.gpsModel.speed) < useGpsForHeadingKnotSpeed;

	getMergedHeading(true); //decrease compass weight on each iteration

    if(m_mockPosition) {
        return position->getHeading();
    }

	if (m_getHeadingFromCompass) {
		//Should return compass heading if below one knot and not currently merging and vice versa
    	return Utility::addDeclinationToHeading(getMergedHeading(gpsForbidden), m_waypointModel.declination);
	}
    return m_systemStateModel.gpsModel.heading;
}

int SailingRobot::getMergedHeading(bool increaseCompassWeight){

	float compassHeadingWeight;
	//Shouldn't be hardcoded
	float tickRate = 0.01;

	int headingCompass = Utility::addDeclinationToHeading(m_systemStateModel.compassModel.heading, m_waypointModel.declination);
	int headingGps = m_systemStateModel.gpsModel.heading;

	if (increaseCompassWeight){
		m_gpsHeadingWeight = m_gpsHeadingWeight - tickRate; //Decrease gps weight
		if (m_gpsHeadingWeight < 0.0) m_gpsHeadingWeight = 0;
	}else{
		m_gpsHeadingWeight = m_gpsHeadingWeight + tickRate;
		if (m_gpsHeadingWeight > 1.0) m_gpsHeadingWeight = 1.0;
	}

	compassHeadingWeight = 1.0 - m_gpsHeadingWeight;

	//Debugging - remove later
	std::cout << "\nHEADING: gps heading weight: " << std::to_string(m_gpsHeadingWeight) << "\n";
	std::cout << "HEADING: Compass percentage = " << std::to_string(compassHeadingWeight) << "\n";
	std::cout << "HEADING: Gps value = " << std::to_string(headingGps) << ", weight: " << std::to_string(headingGps * m_gpsHeadingWeight) << "\n";
	std::cout << "HEADING: Compass value = " << std::to_string(headingCompass) << ", weight: " << std::to_string(headingCompass * compassHeadingWeight) << "\n";

	//Difference calculation
	float diff = ((headingGps - headingCompass) + 180 + 360);
	while (diff > 360) diff -= 360;
	diff -= 180;

	//Merge angle calculation
	int returnValue = 360 + headingCompass + (diff * m_gpsHeadingWeight);
	while (returnValue > 360) returnValue -= 360;

	//Debugging, remove later
	std::cout << "HEADING: Difference in degrees: " << std::to_string(diff) << "\n";
	std::cout << "HEADING: MERGED HEADING: " << std::to_string(returnValue) << "\n";

	return returnValue;

}

void SailingRobot::run() {

	//m_dbHandler->clearLogs();
	m_running = true;
	routeStarted = true;

	double rudderCommand, sailCommand;//,heading = 0, insertScanOnce = 0;

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
	//int checkDBcounter = 0;  

  	WaypointBehaviour waypB(m_dbHandler); 
  	LineFollowBehaviour LineFollowB(m_dbHandler);
	RoutingBehaviour *behave;
	if(usingLineFollow)  //Get which system to use
		behave = &LineFollowB;
	else
		behave = &waypB;
  	behave->init();
	m_httpSync->setWaypointUpdateCallback(behave->setWaypointsChanged);

	while(m_running) {
		timer.reset();


		// if(checkDBcounter > 15)
		// {
		// 	checkDBcounter = 0;
		// 	usingLineFollow = std::stoi(m_dbHandler->retrieveCell("sailing_robot_config", "1", "line_follow"));
		// 	if (usingLineFollow != previousBehaviour){ //If following behaviour changes in database
		// 		if(previousBehaviour)
		// 			behave = &waypB;						
		// 		else									
		// 			behave = &LineFollowB;

		// 		behave->init();
		// 		previousBehaviour = usingLineFollow;
		// 	}
		// }

		//Get data from SystemStateModel to local object
		m_systemState->getData(m_systemStateModel);
		//calc & set TWD
		trueWindDirection = Utility::getTrueWindDirection(m_systemStateModel, twdBuffer, twdBufferMaxSize);

		//Compute the commands to send
		behave->computeCommands(m_systemStateModel,position, trueWindDirection ,m_mockPosition, m_getHeadingFromCompass);


		std::cout << "heading: " << heading << "\n";
		std::cout << "heading ssm compass:" << m_systemStateModel.compassModel.heading<<"\n";

        if (m_mockPosition) {
            position->setCourseToSteer(m_waypointRouting.getCTS());
        }

        position->updatePosition();

		if (m_systemStateModel.gpsModel.online) {
			//calc & set TWD
			double twd = Utility::calculateTrueWindDirection(m_systemStateModel,heading);
			twdBuffer.push_back(twd);// new wind calculation

			//twdBuffer.push_back(heading + windDir);// old wind calculation

			while (twdBuffer.size() > twdBufferMaxSize) {
				twdBuffer.erase(twdBuffer.begin());
			}

			double rudder = 0, sail = 0;
			m_waypointRouting.getCommands(rudder, sail,
				position->getModel(),
				Utility::meanOfAngles(twdBuffer), heading, m_systemStateModel);


			if(m_dbHandler->retrieveCellAsInt("wind_vane_config", "1", "use_self_steering")) {
				if(m_dbHandler->retrieveCellAsInt("wind_vane_config", "1", "wind_sensor_self_steering")) {
					m_windVaneController.setVaneAngle(m_waypointRouting.getTWD(), m_waypointRouting.getCTS());
					// WindVaneController::getAngle() will fetch set angle of vane.
					// To do: Pass angle to engine controlling wind vane.
				} else {
                    double heading = getHeading();

                    m_windVaneController.adjustAngle(heading,m_waypointRouting.getCTS() );
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
		sailCommand = m_sailCommand.getCommand(behave->getSailCommand());
		if(!m_externalCommand->getAutorun()) {
			sailCommand = m_externalCommand->getSailCommand();
		}

		//rudder adjustment
		m_rudderServo.setPosition(rudderCommand);
		//sail adjustment
		m_sailServo.setPosition(sailCommand);

		m_systemState->setRudder(rudderCommand);
		m_systemState->setSail(sailCommand);

		//Save data in database
		behave->manageDatabase(trueWindDirection,m_systemStateModel);

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