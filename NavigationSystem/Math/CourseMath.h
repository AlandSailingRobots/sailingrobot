#ifndef __COURSEMATH_H__
#define __COURSEMATH_H__


#include <stdint.h>


class CourseMath {
public:
	static int16_t calculateBTW(double gpsLon, double gpsLat, double waypointLon, double waypointLat);
	static double calculateDTW(double gpsLon, double gpsLat, double waypointLon, double waypointLat);
private:
	CourseMath() { };
};

#endif
