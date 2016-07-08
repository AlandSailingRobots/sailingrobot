#include "MockGPSReader.h"
#include <math.h>
#include <iostream>
#include "models/GPSModel.h"

MockGPSReader::MockGPSReader() {
	m_timestamp = "";
	m_latitude = 60.103580;
	m_longitude = 19.867784;
	m_altitude = 0;
	m_speed = 0;
	m_heading = 0;
	m_mode = 0;
	m_satellitesUsed = 0;
}

MockGPSReader::~MockGPSReader() {
}


bool MockGPSReader::connectToGPS() {
	return true;
}

bool MockGPSReader::readGPS(int timeout) {
	return true;
}

bool MockGPSReader::isOnline() {
	return true;
}

std::string MockGPSReader::getTimestamp() {
	return m_timestamp;
}

double MockGPSReader::getLatitude() {
	return m_latitude;
}

double MockGPSReader::getLongitude() {
	return m_longitude;
}

double MockGPSReader::getAltitude() {
	return m_altitude;
}

double MockGPSReader::getSpeed() {
	return m_speed;
}

double MockGPSReader::getHeading() {
	return m_heading;
}

int MockGPSReader::getMode() {
	return m_mode;
}

int MockGPSReader::getSatellitesUsed() {
	return m_satellitesUsed;
}

void MockGPSReader::setTimestamp(std::string timestamp) {
	m_timestamp = timestamp;
}

void MockGPSReader::setLatitude(double latitude) {
	m_latitude = latitude;
}

void MockGPSReader::setLongitude(double longitude) {
	m_longitude = longitude;
}

void MockGPSReader::setAltitude(double altitude) {
	m_altitude = altitude;
}

void MockGPSReader::setSpeed(double speed) {
	m_speed = speed;
}

void MockGPSReader::setHeading(double heading) {
	m_heading = heading;
}

void MockGPSReader::setMode(int mode) {
	m_mode = mode;
}

void MockGPSReader::setSatellitesUsed(int satellitesUsed) {
	m_satellitesUsed = satellitesUsed;
}

void MockGPSReader::setDataFromCommands(int rudderCommand, int sailCommand) {
	const float maxSpeed = 10; //meters per iteration
	const float maxTurn = 10;
	if (m_speed > 0 && rudderCommand != 5984) { //ruddercommand != midships
		//rss xtrm-mid 7616,7000,6500,mid5984mid,5468,4968,4352
		if (rudderCommand == 7616) {
			m_heading += maxTurn;
//			m_speed -= (maxSpeed/40);
		}
		if (rudderCommand == 7000) {
			m_heading += maxTurn/2;
//			m_speed -= (maxSpeed/80);
		}
		if (rudderCommand == 6500) {
			m_heading += maxTurn/3;
		}
		if (rudderCommand == 4352) {
			m_heading -= maxTurn;
//			m_speed -= (maxSpeed/40);
		}
		if (rudderCommand == 4968) {
			m_heading -= maxTurn/2;
//			m_speed -= (maxSpeed/80);
		}
		if (rudderCommand == 5468) {
			m_heading -= maxTurn/3;
		}

		if (m_speed < 0) {
			m_speed = 0;
		}
		if (m_heading >= 360) {
			m_heading -= 360;
		}
		if (m_heading < 0) {
			m_heading += 360;
		}
	}

	//sss cls-run 7424,6600,6200,5824
	if (sailCommand == 5824) {
		m_speed += maxSpeed/5;
		if (m_speed > maxSpeed) {
			m_speed = maxSpeed;
		}
	}
	if (sailCommand == 6200) {
		m_speed += maxSpeed/10;
		if (m_speed > maxSpeed) {
			m_speed = maxSpeed;
		}
	}
	if (sailCommand == 6600) {
		m_speed += maxSpeed/20;
		if (m_speed > maxSpeed) {
			m_speed = maxSpeed;
		}
	}
	if (sailCommand == 7424) {
		m_speed += maxSpeed/40;
		if (m_speed > maxSpeed) {
			m_speed = maxSpeed;
		}
	}

	//update lat/long
	float gpsMeter = 0.00001; 
	const double PI = 3.1415926;
	float radHeading = m_heading * (PI/180);
	float speed = gpsMeter * m_speed;
	m_latitude += cos(radHeading) * speed; 
	m_longitude += sin(radHeading) * speed; 
}
GPSModel MockGPSReader::getModel() {
	return GPSModel("",PositionModel(0,0),0,0,0,0);
}
