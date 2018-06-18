#ifndef Utility_h
#define Utility_h

#include <stdint.h>  // uint8_t
#include <array>
#include <iostream>
#include <vector>

class Utility {
   public:
    static int combineBytes(uint8_t MSB, uint8_t LSB);
    static int combineBytesSigned(uint8_t MSB, uint8_t LSB);

    static bool IsOutOfRange(float f);
    static std::vector<float> fixAngles(std::vector<float> v);
    static float getMedianValue(std::vector<float> v);
    static float mean(std::vector<float> values);
    static float meanOfAngles(std::vector<float> anglesInDegrees);
    static int sgn(double value);
    // static int16_t pi(double pGain, double iGain, uint16_t heading, uint16_t desiredHeading);
    static std::vector<double> maxAndIndex(std::vector<double> mylist);

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
    static double limitAngleRange180(double angle);
    static double limitRadianAngleRange(double angle);

    /*
     * Returns the angle value corresponding to x by the linear function defined by the points
     * (x1,angle1) and (x2,angle2). Angles are in degree.
     */
    static double linearFunctionBetweenAngle(double x,
                                             double x1,
                                             double x2,
                                             double angle1,
                                             double angle2);
    static double degreeToRadian(double degrees);
    static double radianToDegree(double radians);

    static int16_t headingDifference(uint16_t h1, uint16_t h2);
    static int16_t signedHeadingDifference(uint16_t h1, uint16_t h2);
    static uint16_t wrapAngle(int16_t angle);

    static int addDeclinationToHeading(int heading, int declination);

    static double directionAdjustedSpeed(double gpsHeading, double compassHeading, double gpsSpeed);

    static double calculateSignedDistanceToLine(const double nextLon,
                                                const double nextLat,
                                                const double prevLon,
                                                const double prevLat,
                                                const double gpsLon,
                                                const double gpsLat);
    static double calculateWaypointsOrthogonalLine(const double nextLon,
                                                   const double nextLat,
                                                   const double prevLon,
                                                   const double prevLat,
                                                   const double gpsLon,
                                                   const double gpsLat);

    static double calculateTrueWindDirection(const int windsensorDir,
                                             const int windsensorSpeed,
                                             const double gpsSpeed,
                                             const double heading);
    static double calculateTrueWindSpeed(int windsensorDir,
                                         int windsensorSpeed,
                                         double gpsSpeed,
                                         double heading);
    static double getTrueWindDirection(int windsensorDir,
                                       int windsensorSpeed,
                                       double gpsSpeed,
                                       int compassHeading,
                                       std::vector<float>& twdBuffer,
                                       const unsigned int twdBufferMaxSize);

    static void calculateApparentWind(const int windsensorDir,
                                      const int windsensorSpeed,
                                      const double gpsSpeed,
                                      const double heading,
                                      const double trueWindDirection,
                                      double& apparentWindSpeed,
                                      double& apparentWindDirection);
    static double getApparentWindSpeed(const int windsensorDir,
                                       const int windsensorSpeed,
                                       const double gpsSpeed,
                                       const double heading,
                                       const double trueWindDirection);
    static double getApparentWindDirection(const int windsensorDir,
                                           const int windsensorSpeed,
                                           const double gpsSpeed,
                                           const double heading,
                                           const double trueWindDirection);

    /*
     * Add two vectors expressed in polar coordinates and return the resulting vector (in polar
     * coordinates). vector = (radius, angle).		Angles are in radian, in [0, 2*PI[.
     */
    static std::vector<double> polarVerctorsAddition(std::vector<double> v1,
                                                     std::vector<double> v2);

    static void addValueToBuffer(float value,
                                 std::vector<float>& buffer,
                                 unsigned int bufferMaxSize);

    static void sphericalCoordinateSystem(const double lat, const double lon, double& x, double& y);
    static void calculateVelocity(const uint16_t course,
                                  const double speed,
                                  double& vX,
                                  double& vY);

    /*
     * Use formula to calculate salinity from marine sensor data
     * http://www.chemiasoft.com/chemd/salinity_calculator
     */
    static float calculateSalinity(const float temperature, const float conductivety);
};

#endif
