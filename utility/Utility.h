#ifndef Utility_h
#define Utility_h

#include <stdint.h> // uint8_t
#include <vector>
#include <array>
#include <cmath>

#define CONVERSION_FACTOR_METER_TO_GPS 0.00000015695406942385 // in rad/meters
#define EARTH_RADIUS 6371000 // in meters

class Utility {
public:
	static int combineBytes(uint8_t MSB, uint8_t LSB);
	static int combineBytesSigned(uint8_t MSB, uint8_t LSB);

	static bool IsOutOfRange (float f);
	static std::vector<float> fixAngles(std::vector<float> v);
	static float getMedianValue(std::vector<float> v);
	static float mean(std::vector<float> values);
	static float meanOfAngles(std::vector<float> anglesInDegrees);
	static int sgn(double value);

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
	static double limitRadianAngleRange(double angle);
	static double degreeToRadian(double degrees);
	static double radianToDegree(double radians);

	static int addDeclinationToHeading(int heading, int declination);

	static double directionAdjustedSpeed(double gpsHeading,double compassHeading,double gpsSpeed);

	static double calculateSignedDistanceToLine(const double nextLon, const double nextLat, const double prevLon, const double prevLat, 
					const double gpsLon, const double gpsLat);
	static double calculateWaypointsOrthogonalLine(const double nextLon, const double nextLat, const double prevLon, const double prevLat, 
					const double gpsLon, const double gpsLat);

	static double calculateTrueWindDirection(const int windsensorDir, const int windsensorSpeed, const double gpsSpeed, const double heading);
	static double calculateTrueWindSpeed(int windsensorDir, int windsensorSpeed, double gpsSpeed, double heading);
	static double getTrueWindDirection(int windsensorDir, int windsensorSpeed, double gpsSpeed, int compassHeading, 
			std::vector<float> &twdBuffer, const unsigned int twdBufferMaxSize);

	static void calculateApparentWind(const int windsensorDir, const int windsensorSpeed, const double gpsSpeed, const double heading, const double trueWindDirection,
		                                         double &apparentWindSpeed, double &apparentWindDirection);
	static double getApparentWindSpeed(const int windsensorDir, const int windsensorSpeed, const double gpsSpeed, const double heading, const double trueWindDirection);
	static double getApparentWindDirection(const int windsensorDir, const int windsensorSpeed, const double gpsSpeed, const double heading, const double trueWindDirection);

    /**
     * Gives the sum between two angles regardless of their definition.
     * It's radAngle1+radAngle2.
     * The angles needs to be in radians.
     * @param radAngle1
     * @param radAngle2
     * @return
     */
    static double wrapToPi(double angleRad1, double angleRad2);
    /**
     * \Brief Compute the distance between 2 GPS points
     * Haversine algorithm for distance computation on Earth. \n
     * Took on http://www.movable-type.co.uk/scripts/latlong.html \n
     * a = sin²(Δφ/2) + cos φ1 ⋅ cos φ2 ⋅ sin²(Δλ/2) \n
     * c = 2 ⋅ atan2( √a, √(1−a) ) \n
     * distance = Rearth ⋅ c \n
     * @param point1X
     * @param point1Y
     * @param point2X
     * @param point2Y
     * @return double          The distance between the two gps points
     */
    static double calculateGPSDistance(double point1X,
                                       double point1Y,
                                       double point2X,
                                       double point2Y);

	/*
	Return an angle between 0 and 2*M_PI.
	double theta : (IN) Value.
	Return : The converted angle.
	*/
	static inline double fmod_2PI_pos(double theta)
	{
		return fmod(fmod(theta, 2*M_PI)+2*M_PI, 2*M_PI);
	}

	/*
	Return an angle between -M_PI and M_PI.
	double theta : (IN) Value.
	Return : The converted angle.
	*/
	static inline double fmod_2PI(double theta)
	{
		return fmod(fmod(theta, 2*M_PI)+3*M_PI, 2*M_PI)-M_PI;
	}

    double wrapToPi(
            double radAngle1,
            double radAngle2);
};

#endif
