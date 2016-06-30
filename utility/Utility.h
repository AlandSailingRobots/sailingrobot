#ifndef Utility_h
#define Utility_h

#include <stdint.h> // uint8_t
#include <vector>
#include "models/SystemStateModel.h"


class Utility {
public:
	static int combineBytes(uint8_t MSB, uint8_t LSB);
	static int combineBytesSigned(uint8_t MSB, uint8_t LSB);

	static bool IsOutOfRange (float f);
	static std::vector<float> fixAngles(std::vector<float> v);
	static float getMedianValue(std::vector<float> v);
	static float mean(std::vector<float> values);
	static float meanOfAngles(std::vector<float> anglesInDegrees);
	
	/*
	 * Converts an angle in degrees to cartesian coordinates (x,y) on the
	 * unit circle
	 */
	static void polarToCartesian(float degrees, float& x, float& y);

	/*
	 * Check if angle is between sectorAngle1 and sectorAngle2, going from 1 to 2 clockwise
	 */
	static bool isAngleInSector(double angle, double sectorAngle1, double sectorAngle2);
	static double angleDifference(double angle1, double angle2);
	static double limitAngleRange(double angle);
	static double degreeToRadian(double degrees);
	static double radianToDegree(double radians);

	static int addDeclinationToHeading(int heading, int declination);

	static double directionAdjustedSpeed(double gpsHeading,double compassHeading,double gpsSpeed);

	static double calculateTrueWindDirection(const SystemStateModel& systemStateModel , double heading);
	
};

#endif