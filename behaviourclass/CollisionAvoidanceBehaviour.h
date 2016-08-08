//
// Created by tag on 12/07/16.
//

#ifndef SAILINGROBOT_TEST_AVOIDANCE_BEHAVIOUR_H
#define SAILINGROBOT_TEST_AVOIDANCE_BEHAVIOUR_H

#include <vector>
#include <math.h>
#include <stdlib.h>
#include "libs/Eigen/Dense"
#include "utility/Utility.h"
#include <boost/geometry/geometry.hpp>

// TODO : Receive these values from the database or from RoutingBehaviour
// Some of these values should be available for all behaviour classes
#define DISTANCE_NOT_THE_SAME_OBSTACLE 15.0
#define MAXIMUM_SENSOR_RANGE 100.0
#define SENSOR_HEADING_RELATIVE_TO_BOAT 0.0 // There might be several sensors
#define SENSOR_ARC_ANGLE 90.0 // Every angle is in radian
#define CHANNEL_RADIUS 15.0 // in meters
#define SAFE_DISTANCE 30.0 // in meters
#define EARTH_RADIUS 6371000 // in meters
#define CONVERSION_FACTOR_METER_TO_GPS 0.00000015695406942385 // in rad/meters

#define FIND_CENTER_NORMAL 0
#define FIND_CENTER_DETECTED 1
// SAVED CODE IN CASE OF ARCHITECTURAL CHANGE
/*

//struct sensorData{
//    double gpsLon; //x
//    double gpsLat; //y
//    double compass;
//    double windDirection;
//};

*/

typedef boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian> boostPoint;
typedef boost::geometry::model::polygon<boostPoint> boostPolygon;

/**
 * Structure which contains every obstacle data from the sensors
 */
struct ObstacleData {
    double minDistanceToObstacle;
    double maxDistanceToObstacle; /**< -1 = infinite */
    double leftBoundHeadingRelativeToBoat;
    double rightBoundHeadingRelativeToBoat;
};
/**
 * Structure which contains every obstacle data
 */
struct Obstacle {
    //Polygon
    std::vector<Eigen::Vector2d> polygon;
    Eigen::Vector2d center;
    bool upToDate;
    std::string color;
};
/**
 * Structure which contains every used data from the sensors.
 */
struct SensorData {
    Eigen::Vector2d gpsPos;
    double gpsSpeed;
    double compHeading;
    double gpsHeading;
    double windDirection;
    double windSpeed; /**< meter par second */
    int pitch;
    int roll;

    //TODO : put the real inputs here (video, sonar or whatever)
    std::vector<ObstacleData> detectedObstacles;
};
struct FollowedLine {
    Eigen::Vector2d startPoint;
    Eigen::Vector2d endPoint;
};
struct CommandOutput {
    double deltaRudder;
    double deltaSail;
};
struct MinPotField {
    double x;
    double y;
    double row;
    double col;
}; //Trash, not used. Could be useful though
struct Simulation {
    bool waypoints;
    bool obstacles;
};
struct PotentialMap{
    const double xMin;
    const double xMax;
    const double yMin;
    const double yMax;
    Eigen::ArrayXXd field;
};
struct BearingOnlyVars{
    bool only_direction_mode;
    bool have_to_avoid_obstacle;
};

/**
 * Collision avoidance class
 * Super class call. Calls the database
 */
class CollisionAvoidanceBehaviour : public RoutingBehaviour {
    //For test only
public:
    CollisionAvoidanceBehaviour(DBHandler *db);
    ~CollisionAvoidanceBehaviour() {};

    /**
     * Test if everything is ok before starting everything
     * @return
     */
    bool init(); //

    /**
     * Compute the commands
     * Too much output parameters, should be reduced.
     *
     * This is a trick to interface this code and the rest of the c++ code
     * @param systemStateModel
     * @param position
     * @param trueWindDirection
     * @param mockPosition
     * @param getHeadingFromCompass
     */
    void computeCommands(SystemStateModel &systemStateModel,
                         std::unique_ptr<Position> const &position,
                         double trueWindDirection,
                         bool mockPosition,
                         bool getHeadingFromCompass);
    void manageDatabase(double trueWindDirection, SystemStateModel &systemStateModel);

private:

    std::vector<Obstacle> m_seenObstacles;
    // Since followedLine is updated from the waypointNode at each iteration
    // maybe it's not necessary to put it as a class variable.
    // TODO : investigate usefulness of followedLine as a class variable, same for sensorOutput
    FollowedLine m_followedLine;
    std::vector<Eigen::Vector2d> m_sailingZone;
    SensorData m_sensorOutput;

    bool m_tack; //Need init
    int m_tackingDirection; //Need init
    BearingOnlyVars m_bearingOnlyVars; // TODO use these variables

    // UTILITY FUNCTIONS
    /*
     * Most of them are functions to handle geometry.
     */

    /**
     * Gives the difference between two angles regardless of their definition.
     * It's radAngle1-radAngle2.
     * The angles needs to be in radians.
     * I don't trust the one from Utility
     * @param radAngle1
     * @param radAngle2
     * @return
     */
    double angleDiff(double radAngle1,
                     double radAngle2);

    /**
     * \Brief Compute the distance between 2 GPS points
     * Haversine algorithm for distance computation on Earth. \n
     * Took on http://www.movable-type.co.uk/scripts/latlong.html \n
     * a = sin²(Δφ/2) + cos φ1 ⋅ cos φ2 ⋅ sin²(Δλ/2) \n
     * c = 2 ⋅ atan2( √a, √(1−a) ) \n
     * distance = Rearth ⋅ c \n
     * @return
     */
    double calculateGPSDistance(Eigen::Vector2d, Eigen::Vector2d);

    /**
     * This is the half-way point along a great circle path between the two points.
     * Got from http://www.movable-type.co.uk/scripts/latlong.html
     * @param pt1
     * @param pt2
     * @return
     */
    Eigen::Vector2d findMidPoint(Eigen::Vector2d pt1, Eigen::Vector2d pt2);

    /**
     * Find the center of a polygon defined anticlockwise.
     *
     * 2 options
     * FIND_CENTER_DETECTED = 1 : for detected obstacles
     * FIND_CENTER_NORMAL = 0 : for other polygons, default parameter
     * @param polygon
     * @param option
     * @return
     */
    Eigen::Vector2d findCenter(const std::vector<Eigen::Vector2d> polygon,
                               int option=0);

    /**
     * Gives the area in unit of the vector (squared of course).
     * So be careful of the units of the values inside the vectors.
     * Computes the area as a sum of spherical triangles.
     *
     * Uses https://en.wikipedia.org/wiki/Spherical_trigonometry#Lines%5Fon%5Fa%5Fsphere
     *
     * The polygon must be initialized anticlockwise
     * @param polygon
     * @return
     */
    double getArea(std::vector<Eigen::Vector2d> polygon);

    /**
     * Get the initial bearing from a start point toward another point
     * on the surface of the earth.
     *
     * Got from http://www.movable-type.co.uk/scripts/latlong.html
     *
     * @param pt1 (Longitude in radians, Latitude in radians)
     * @param pt2 (Longitude in radians, Latitude in radians)
     * @return
     */
    double getInitialBearing(Eigen::Vector2d pt1,Eigen::Vector2d pt2);

    /**
     * Compute the signed distance between a line and a point
     * Uses eigen::vector2d in GPS coordinates
     * Got from http://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment
     * and implemented it for this code.
     * @param linePt1
     * @param linePt2
     * @param point
     * @return
     */
    double signedDistanceFromLine(Eigen::Vector2d linePt1,
                                  Eigen::Vector2d linePt2,
                                  Eigen::Vector2d point);

    /**
     * Get the closest vertex between the given polygon and a point.
     *
     * The coordinates of the points of the polygon are in latitude and longitude
     * @param polygon
     * @param point
     * @return
     */
    Eigen::Vector2d getClosestVertex(std::vector<Eigen::Vector2d> polygon,
                                     Eigen::Vector2d point);

    /**
     * Get the closest point between the given polygon and a point.
     *
     * the coordinates of the points of the polygon are in latitude and longitude
     * @param polygon
     * @param point
     * @return
     */
    Eigen::Vector2d getClosestPoint(std::vector<Eigen::Vector2d> polygon,
                                    Eigen::Vector2d point);

    /**
     * Got from http://www.movable-type.co.uk/scripts/latlong.html
     * Given a start point, initial bearing, and distance, this will calculate
     * the destination point and final bearing travelling along
     * a (shortest distance) great circle arc.
     *
     * Formula: 	φ2 = asin( sin φ1 ⋅ cos δ + cos φ1 ⋅ sin δ ⋅ cos θ )
	 *              λ2 = λ1 + atan2( sin θ ⋅ sin δ ⋅ cos φ1, cos δ − sin φ1 ⋅ sin φ2 )
     *
     * where 	φ is latitude, (y)
     *          λ is longitude, (x)
     *          θ is the bearing (clockwise from north),
     *          δ is the angular distance d/R;
     *          d being the distance travelled,
     *          R the earth’s radius
     *
     * @param distance
     * @param bearing
     * @param startPoint
     * @return
     */
    Eigen::Vector2d getPointWithDistanceAndBearing(double distance,
                                                   double bearing,
                                                   Eigen::Vector2d startPoint);

    /**
     * Transform GPS coordinates 2d vectors into 3D cartesian vectors
     * The earth is seen as a sphere.
     * @param vector
     * @return
     */
    Eigen::Vector2d cartesianToLatLon(Eigen::Vector3d vector);

    /**
     * Transform 3D cartesian vectors into GPS coordinates into 2d vectors
     * The origin is the center of the Earth. The earth is seen as a sphere in the computations
     * Error of 0.3% with a spherical model of the earth.
     * @param vector
     * @return
     */
    Eigen::Vector3d latLonToCartesian(Eigen::Vector2d vector);

    /**
     * Distance from a segment on a sphere (the earth).
     * Got from Robotic Sailing 2012 by Colin Sauzé and James Finnis
     *
     * output a std::vector<double>
     *  the distance
     *  from what
     *      0 : the first point
     *      1 : the second point
     *      2 : a point in the line segment
     *
     *  Uses getClosestPoint
     * @param segmentPoint1
     * @param segmentPoint2
     * @param point
     * @return
     */
    std::vector<double> distanceFromSegment(Eigen::Vector2d segmentPoint1,Eigen::Vector2d segmentPoint2,Eigen::Vector2d point);

    /**
     * Verify if the projection of a point on a plane is inside a slice of Earth
     * Got from
     * http://math.stackexchange.com/questions/544946/determine-if-projection-of-3d-point-onto-plane-is-within-a-triangle
     *
     * The first point of the triangle is the origin of the slice.
     * Ouput an error if this is not a slice (left side not the same length as right side)
     * Every polygon in this program is defined anti-clockwise
     *
     * The function verify only if the projection of the point is inside an infinite triangle.
     * So the segment shouldn't be larger than half the permimeter of the earth.
     * @param triangle
     * @param point
     * @return
     */
    bool projectionInsideSlice(const Eigen::Vector3d triangle[],
                               Eigen::Vector3d point);

    /**
     * Convert the polygon of an obstacle to a c++ boost polygon
     * @param obstacle
     * @return
     */
    boostPolygon eigenPolyToBoostPoly(Obstacle obstacle);

    /**
     * Update the polygon of an obstacle with the data inside a boost polygon
     *
     * I make the assumption that there will be only one poly in output
     * Be aware that some segfault might come from here.
     * But it shouln't be the case, no poly is concave here.
     * @param poly
     * @return
     */
    Obstacle updateObstacleWithBoostPoly(Obstacle obstacle,boostPolygon poly);

    /**
     * Clean the obstacles that should have been seen but are not.
     * There is a distance of non detection to set up.
     * @param sensorData
     * @param seenObstacles
     * @return
     */
    std::vector<Obstacle> cleanObstacles(SensorData sensorData,
                                         std::vector<Obstacle> seenObstacles);

    /**
     * register and convert the new obstacles inside the detectedObstacles to a new
     * std::vector<Obstacle>
     * @param sensorData
     * @param seenObstacles
     * @return
     */
    std::vector<Obstacle> registerObstacles(SensorData sensorData,
                                            std::vector<Obstacle> seenObstacles);

    /**
     * merge two set of obstacles together.
     * It outputs the intersection between each obstacles of the set
     * @param sensorData
     * @param seenObstacles
     * @return
     */
    std::vector<Obstacle> mergeObstacles(std::vector<Obstacle> upToDateObstacles,
                                         std::vector<Obstacle> seenObstacles);

    //PRIVATE MAIN FUNCTIONS

    /**
     * Makes the interface between the old code and the new one. This is make the code more modular
     * since changes in the architecture might come.
     * @param systemStateModel
     * @param sim
     * @return
     */
    SensorData update_sensors(SystemStateModel &systemStateModel,
                              Simulation sim);

    /**
     * Is there any obstacles ? If yes, which information can i gather on them.
     * OUTPUT a list of obstacles (struct) with all their characteristics.
     *
     * For now transform hardcoded obstacles to something more usable.
     * @param sensorData
     * @return
     */
    std::vector<Obstacle> check_obstacles(SensorData sensorData, std::vector<Obstacle> seenObstacles);

    /**
     * Check if there is intersection between
     * the current path+security radius and the obstacle
     * @param seenObstacles
     * @return
     */
    bool these_obstacles_are_a_problem(std::vector<Obstacle> seenObstacles);

    /**
     * Compute the potential field for the obstacles, the sailing zone,
     * the boat, the objective, and the wind.
     *
     * For now it creates the matrix at each loop (easier for code review)
     * The size of the matrix needs to be adapted to the size of the sailing zone
     * (with the max and mins for example)
     *
     * Maybe we could create everything in the init function and put it in a
     * struct. This struct would be a class variable. // TODO potField class variable
     *
     * Maybe we should as well put the computation of the potential field in another class
     * due to the length of the function. // TODO see if a new class is needed
     * @param seen_obstacles
     * @param sailing_zone
     * @param followedLine
     * @return
     */
    PotentialMap compute_potential_field(std::vector<Obstacle> seen_obstacles,
                                         std::vector<Eigen::Vector2d> sailing_zone,
                                         FollowedLine followedLine);

    /**
     * Find the minimum in the potential field and return its coordinates in the matrix
     * as well as its real gps coordinates.
     * @param Potential_field
     * @return
     */
    Eigen::Vector2d find_minimum_potential_field(PotentialMap PotentialField);

    /**
     * Gives the new line to follow. It would be better if it added a WP in the
     * DataBase as well.
     *
     * It add 3 intermediate wp to follow and follow them. Everything will be updated
     * on the next loop by update waypoint. Then the boat will continue to follow
     * these waypoints.
     * @param min
     * @return
     */
    FollowedLine compute_new_path(Eigen::Vector2d collision_avoidance_point);

    /**
     * Compute the commands according to the new path.
     * Follow the line between the last waypoint and the next
     * @param line
     * @return
     */
    CommandOutput compute_commands(FollowedLine line);

    /**
     * The most important function of the class, it calls al the others.
     *
     * Some inputs of the function are class variables. This is to improve
     * the readability of the code and its modularity in case of architectural
     * modifications.
     *
     * I don't think inputing system state into this function is the best idea.
     * But update_sensors need it. Either I move update_sensors out of run(), in
     * that case isn't any more a modular function since it lacks update_sensors,
     * or systemStateModel stays where it is for the same result.
     * @param systemStateModel
     * @return
     */
    CommandOutput run(SystemStateModel &systemStateModel);
};

#endif //SAILINGROBOT_TEST_AVOIDANCE_BEHAVIOUR_H