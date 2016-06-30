#ifndef __GPS_H__
#define __GPS_H__

#include <string>

class GPSModel;

class GPS {

public:
	/*Constructor*/
	GPS() {};

	/*Destructor*/
	virtual ~GPS() {};

	/*Sets up a connection to the USB-connected GPS*/
	virtual void connectToGPS()=0;

	/*reads data from the GPS given a number of attempts and timeout for each attempt*/
	virtual void readGPS(int timeout)=0;

	/*Returns true if gps is online.*/
	virtual bool isOnline()=0;

	/*Returns the latest updated timestamp by the GPS*/
	virtual std::string getTimestamp()=0;

	/*Returns the latest updated latitude by the GPS*/
	virtual double getLatitude()=0;

	/*Returns the latest updated longitude by the GPS*/
	virtual double getLongitude()=0;

	/*Returns the latest updated altitude by the GPS - Only accurate if mode is 3*/
	virtual double getAltitude()=0;

	/*Returns the latest updated speed by the GPS*/
	virtual double getSpeed()=0;

	/*Returns the latest updated heading by the GPS - North is 0 degrees, 90 degrees is east*/
	virtual double getHeading()=0;

	/*Returns the latest updated mode by the GPS - Value 3 is the best*/
	virtual int getMode()=0;

	/*Returns the latest updated number of satellites by the GPS that the GPS has a connection to*/
	virtual int getSatellitesUsed()=0;

	/*Returns model*/
	virtual GPSModel getModel()=0;
};

#endif
