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

	//dbhandler
	m_dbHandler.openDatabase("db");
	m_dbHandler.createTables();
std::cout << "dbh inited\n";

	//servos
	std::string val = m_dbHandler.retriveCell("configs", "1", "mc_portname");
	m_maestroController.setPort(val.c_str());
std::cout << "maestro inited\n";
	m_rudderServo.setController(&m_maestroController);
	m_rudderServo.setChannel(0);
	m_rudderServo.setMin(4000);
	m_rudderServo.setMax(8000);
	m_rudderServo.setSpeed(0);
	m_rudderServo.setAcceleration(0);

	m_sailServo.setController(&m_maestroController);
	m_sailServo.setChannel(1);
	m_sailServo.setMin(4000);
	m_sailServo.setMax(8000);
	m_sailServo.setSpeed(0);
	m_sailServo.setAcceleration(0);

	m_windSensor.setController(&m_maestroController);
	m_windSensor.setChannel(5);

std::cout << "servos inited\n";
	//windsensor
	try {
		m_windSensorController.loadConfig("CV7", "/dev/ttyAMA0", 4800);
	} catch(const char* exception) {
		cout << exception << endl;
		return;
	}
	//m_windSensorController.mockDirection(180);
std::cout << "windsens inited\n";



	//gps
	m_gpsReader.connectToGPS("/dev/ttyUSB0", "localhost");
std::cout << "gpsr inited\n";

	//coursecalc
	m_courseCalc.setTACK_ANGLE(45);
	m_courseCalc.setSECTOR_ANGLE(5);
std::cout << "coursecalc inited\n";

	//waypoints
	m_waypointList.add(100.1, 30.1);
	m_waypointList.add(200.1, 40.1);
std::cout << "wp inited\n";


}


void SailingRobot::run() {
	
	for (int i = 0; i < 100; i++) {

//		if (i > 10) {
//			m_windSensorController.mockDirection(0);
//		}
		
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
