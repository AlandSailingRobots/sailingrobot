#include "CourseMath.h"
#include <cmath>
#include "models/PositionModel.h"
#include "utility/Utility.h"


double CourseMath::calculateBTW(PositionModel boat, PositionModel waypoint) const
{
	double boatLatitudeInRadian = Utility::degreeToRadian(boat.latitude);
	double waypointLatitudeInRadian = Utility::degreeToRadian(waypoint.latitude);
	double deltaLongitudeRadian = Utility::degreeToRadian(waypoint.longitude - boat.longitude);

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


double CourseMath::calculateDTW(PositionModel boat, PositionModel waypoint) const
{
	const double radiusOfEarth = 6371.0;

	double deltaLatitudeRadians = Utility::degreeToRadian(waypoint.latitude - boat.latitude);
	double boatLatitudeInRadian = Utility::degreeToRadian(boat.latitude);
	double waypointLatitudeInRadian = Utility::degreeToRadian(waypoint.latitude);
	double deltaLongitudeRadians = Utility::degreeToRadian(waypoint.longitude - boat.longitude);

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

double CourseMath::calculateDTW(float boatLon, float boatLat, float waypointLon, float waypointLat) const
{
	const double radiusOfEarth = 6371.0;

	double deltaLatitudeRadians = Utility::degreeToRadian(waypointLat - boatLat);
	double boatLatitudeInRadian = Utility::degreeToRadian(boatLat);
	double waypointLatitudeInRadian = Utility::degreeToRadian(waypointLat);
	double deltaLongitudeRadians = Utility::degreeToRadian(waypointLon - boatLon);

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