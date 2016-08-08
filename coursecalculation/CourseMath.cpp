#include "CourseMath.h"
#include <cmath>
#include "utility/Utility.h"


double CourseMath::calculateBTW(double gpsLon, double gpsLat, double waypointLon, double waypointLat) const
{
	double boatLatitudeInRadian = Utility::degreeToRadian(gpsLat);
	double waypointLatitudeInRadian = Utility::degreeToRadian(waypointLat);
	double deltaLongitudeRadian = Utility::degreeToRadian(waypointLon - gpsLon);

	double y_coordinate = sin(deltaLongitudeRadian)
						* cos(waypointLatitudeInRadian);

	double x_coordinate = cos(boatLatitudeInRadian)
						* sin(waypointLatitudeInRadian)
						- sin(boatLatitudeInRadian)
						* cos(waypointLatitudeInRadian)
						* cos(deltaLongitudeRadian);

	double bearingToWaypointInRadian = atan2(y_coordinate, x_coordinate);
	double bearingToWaypoint = Utility::radianToDegree(bearingToWaypointInRadian);

	bearingToWaypoint = Utility::limitAngleRange(bearingToWaypoint);

	return bearingToWaypoint;
}

double CourseMath::calculateDTW(double gpsLon, double gpsLat, double waypointLon, double waypointLat) const
{
	const double radiusOfEarth = 6371.0;

	double deltaLatitudeRadians = Utility::degreeToRadian(waypointLat - gpsLat);
	double boatLatitudeInRadian = Utility::degreeToRadian(gpsLat);
	double waypointLatitudeInRadian = Utility::degreeToRadian(waypointLat);
	double deltaLongitudeRadians = Utility::degreeToRadian(waypointLon - gpsLon);

	double a = sin(deltaLatitudeRadians/2)
			* sin(deltaLatitudeRadians/2)
			+ cos(boatLatitudeInRadian)
			* cos(waypointLatitudeInRadian)
			* sin(deltaLongitudeRadians/2)
			* sin(deltaLongitudeRadians/2); 			

	double b = 2 * atan2(sqrt(a), sqrt(1 - a));
	double distanceToWaypoint = radiusOfEarth * b * 1000;
	
	return distanceToWaypoint;
}
