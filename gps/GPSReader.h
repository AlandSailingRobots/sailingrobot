#ifndef __GPSREADER_H__
#define __GPSREADER_H__

#include <string>
#include <libgpsmm.h>
#include "GPS.h"
#include "models/PositionModel.h"
#include "models/GPSModel.h"
#include "logger/Logger.h"


/**---------------------------------------
 *	USAGE:
 * 
 *	GPSReader g;
 *	g.connectToGPS(string portName, string connectionName);
 *	g.readGPS();
 * 
 *----------------------------------------*/

class GPSReader: public GPS {

private:
	GPSModel m_model;

	int m_mode;
	gpsmm * m_gpsConnection;

	std::string secondsToTimeStamp(double seconds, bool utc);
	std::string parseDate(int year, int mon, int day);
	std::string parseTime(int hour, int min, int sec);
	std::string parseDateTime(int year, int mon, int day, int hour, int min, int sec);

public:
	/*Constructor*/
	GPSReader();

	/*Destructor*/
	~GPSReader();

	/*Sets up a connection to the USB-connected GPS*/
	bool connectToGPS();

	/*reads data from the GPS given a number of attempts and timeout for each attempt*/
	bool readGPS(int timeout);

	/*Returns true if gps is online.*/
	bool isOnline();

	/*Returns the latest updated timestamp by the GPS*/
	std::string getTimestamp();

	/*Returns the latest updated latitude by the GPS*/
	double getLatitude();

	/*Returns the latest updated longitude by the GPS*/
	double getLongitude();

	/*Returns the latest updated altitude by the GPS - Only accurate if mode is 3*/
	double getAltitude();

	/*Returns the latest updated speed by the GPS*/
	double getSpeed();

	/*Returns the latest updated heading by the GPS - North is 0 degrees, 90 degrees is east*/
	double getHeading();

	/*Returns the latest updated mode by the GPS - Value 3 is the best*/
	int getMode();

	/*Returns the latest updated number of satellites by the GPS that the GPS has a connection to*/
	int getSatellitesUsed();

	/*Returns model*/
	GPSModel getModel();
};

#endif
