#ifndef __MOCKGPSREADER_H__
#define __MOCKGPSREADER_H__

#include <string>
#include "GPS.h"

class GPSModel;

class MockGPSReader: public GPS {

private:
	std::string m_timestamp;
	double m_latitude;
	double m_longitude;
	double m_altitude;
	double m_speed;
	double m_heading;
	int m_mode;
	int m_satellitesUsed;

public:

	/*Constructor*/
	MockGPSReader();

	/*Destructor*/
	~MockGPSReader();

	/*Sets up a connection to the USB-connected GPS*/
	bool connectToGPS();

	/*reads data from the GPS given a number of attempts and timeout for each attempt*/
	bool readGPS(int timeout);

	/*Returns true if gps is online.*/
	bool isOnline();

	/*Returns the latest updated timestamp by the GPS*/
	std::string getTimestamp();
	void setTimestamp(std::string timestamp);

	/*Returns the latest updated latitude by the GPS*/
	double getLatitude();
	void setLatitude(double latitude);

	/*Returns the latest updated longitude by the GPS*/
	double getLongitude();
	void setLongitude(double longitude);

	/*Returns the latest updated altitude by the GPS - Only accurate if mode is 3*/
	double getAltitude();
	void setAltitude(double altitude);

	/*Returns the latest updated speed by the GPS*/
	double getSpeed();
	void setSpeed(double speed);

	/*Returns the latest updated heading by the GPS - North is 0 degrees, 90 degrees is east*/
	double getHeading();
	void setHeading(double heading);

	/*Returns the latest updated mode by the GPS - Value 3 is the best*/
	int getMode();
	void setMode(int mode);

	/*Returns the latest updated number of satellites by the GPS that the GPS has a connection to*/
	int getSatellitesUsed();
	void setSatellitesUsed(int satellitesUsed);

	void setDataFromCommands(int rudderCommand, int sailCommand);

	GPSModel getModel();
};

#endif
