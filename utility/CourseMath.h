#ifndef __COURSEMATH_H__
#define __COURSEMATH_H__


class CourseMath {
public:
	static double calculateBTW(double gpsLon, double gpsLat, double waypointLon, double waypointLat);
	static double calculateDTW(double gpsLon, double gpsLat, double waypointLon, double waypointLat);
private:
	CourseMath() { };
};

#endif
