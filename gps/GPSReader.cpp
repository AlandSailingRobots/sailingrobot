#include "GPSReader.h"
#include <sstream>
#include <cmath>
#include <cstdlib>
#include <iostream>

GPSReader::GPSReader() :
	m_model(GPSModel("",PositionModel(0,0),0,0,0,0))
{
	m_mode = 0;
	m_gpsConnection = NULL;
}

GPSReader::~GPSReader() {
	if (m_gpsConnection != NULL) {
		delete m_gpsConnection;
	}
}

// gmt + 3 h timestamp (helsinki)
std::string GPSReader::secondsToTimeStamp(double seconds, bool utc) {
	time_t fullSeconds = static_cast<time_t>(seconds);
	struct tm * timeinfo;
	if(utc) // utc
		timeinfo = gmtime(&fullSeconds);		
	else // gmt + 3
		timeinfo = localtime(&fullSeconds);
		
	char timeInfoBuffer[100];
	strftime(timeInfoBuffer, 100, "%F %T", timeinfo);
	std::string timeStamp(timeInfoBuffer);
	return timeStamp;
}

void GPSReader::connectToGPS() {

	//system(("sudo pkill gpsd; gpsd " + portName).c_str());
	//system("sudo gpsd /dev/ttyUSB0 -F /var/run/gpsd.sock");

	m_gpsConnection = new gpsmm("localhost", DEFAULT_GPSD_PORT);
	if (m_gpsConnection->stream(WATCH_ENABLE | WATCH_JSON) == NULL) {

		throw "GPSReader::connectToGPS(), unable to connect to GPS.";
	}
}

void GPSReader::readGPS(int timeout) {
	struct gps_data_t* newdata;

	if (!m_gpsConnection->waiting(timeout)) {
		throw "GPSReader::readGPS(), timeout.";
	}

	if ((newdata = m_gpsConnection->read()) == NULL) {
		throw "GPSReader::readGPS(), read error.";
	} else {

		//* Get status flags
		unsigned long int flags = newdata->set;

		//* If ONLINE flag is set
		if((flags & ( 1 << 1 )) >> 1)
		{
			m_model.online = true;
		}

		//* If TIME flag is set
		if((flags & ( 1 << 2 )) >> 2)
		{
			/********************************************* setting up timestamps *****************************/ 
			m_model.online = true;
			m_model.timestamp = secondsToTimeStamp(newdata->fix.time, false);
			m_model.utc_timestamp = secondsToTimeStamp(newdata->fix.time, true);
		}

		//* If LATLON flag is set
		if((flags & ( 1 << 4 )) >> 4)
		{
			m_model.online = true;
			m_model.positionModel.latitude = newdata->fix.latitude;
			m_model.positionModel.longitude = newdata->fix.longitude;
		}

		//* If SPEED flag is set
		if((flags & ( 1 << 6 )) >> 6)
		{
			m_model.speed = newdata->fix.speed;
		}

		//* If TRACK flag is set
		if((flags & ( 1 << 7 )) >> 7)
		{
			m_model.heading = newdata->fix.track;
		}

		//* If SATELLITE flag is set
		if((flags & ( 1 << 15 )) >> 15)
		{
			m_model.satellitesUsed = newdata->satellites_used;
		}

		//* If MODE flag is set
		if((flags & ( 1 << 10 )) >> 10)
		{
			m_mode = newdata->fix.mode;
		}

	}
}

bool GPSReader::isOnline() {
	return m_model.online;
}

std::string GPSReader::getTimestamp() {
	return m_model.timestamp;
}

double GPSReader::getLatitude() {
	return m_model.positionModel.latitude;
}

double GPSReader::getLongitude() {
	return m_model.positionModel.longitude;
}

double GPSReader::getAltitude() {
	return m_model.altitude;
}

double GPSReader::getSpeed() {
	return m_model.speed;
}

double GPSReader::getHeading() {
	return m_model.heading;
}

int GPSReader::getMode() {
	return m_mode;
}

int GPSReader::getSatellitesUsed() {
	return m_model.satellitesUsed;
}

GPSModel GPSReader::getModel() {
	return m_model;
}
