#include <iostream>
#include <iomanip>
#include "gps/GPSReader.h"


int main() {
	GPSReader g;
	try {
		g.connectToGPS();
	} catch(const char* msg) {
		std::cout << msg << std::endl;
	}

	while(true) {
		g.readGPS(50000000);

		std::cout << "online:" << g.isOnline() << ", lat: " << g.getLatitude() << ", long: " << g.getLongitude()
				 << ", heading: " << g.getHeading() << ", speed: " << g.getSpeed()
				 << ", sat: " << g.getSatellitesUsed() << ", time: " << g.getTimestamp()
				 << std::endl << std::endl;
	}
}
 
