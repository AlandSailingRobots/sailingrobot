//
// Created by tag on 12/07/16.
//

#pragma once

#ifndef SAILINGROBOT_TEST_AVOIDANCE_BEHAVIOUR_H
#define SAILINGROBOT_TEST_AVOIDANCE_BEHAVIOUR_H

#include "Nodes/Node.h"
#include "Messages/VesselStateMsg.h"
#include "Messages/ObstacleVectorMsg.h"
#include "Messages/WaypointDataMsg.h"
#include "Messages/CollisionAvoidanceMsg.h"
#include <vector>
#include <math.h>
#include <stdlib.h>
#include <iostream>
#include "libs/Eigen/Dense"
#include "utility/Utility.h"
#include <boost/geometry/geometry.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/multi/geometries/multi_polygon.hpp>
#include "tests/collisionAvoidance/vibes.h"

// TODO : Receive these values from the database or from RoutingBehaviour
// Some of these values should be available for all behaviour classes
#define DISTANCE_NOT_THE_SAME_OBSTACLE 15.0
#define MAXIMUM_SENSOR_RANGE 100.0
#define SENSOR_HEADING_RELATIVE_TO_BOAT 0.0 // There might be several sensors
#define SENSOR_ARC_ANGLE M_PI/3 // Every angle is in radian
#define CHANNEL_RADIUS 15.0 // in meters
#define SAFE_DISTANCE 50.0 // in meters

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
typedef boost::geometry::model::multi_polygon<boostPolygon> boostMultiPolygon;

/**
 * Structure which contains every obstacle data from the sensors
 */
//struct ObstaclesData {
//    double minDistanceToObstacle;
//    double maxDistanceToObstacle; /**< -1 = infinite */
//    double leftBoundHeadingRelativeToBoat;
//    double rightBoundHeadingRelativeToBoat;
//}; //TODO : ObstacleData changed into ObstaclesData
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
    double speed;
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
/**
 * Commands output in radians
 */
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
    double xMin;
    double xMax;
    double yMin;
    double yMax;
    Eigen::ArrayXXd field;
};
/**
 * The datas output by this node
 */
struct WaypointOutput{
    Eigen::Vector2d startPoint;
    Eigen::Vector2d midPoint;
    Eigen::Vector2d endPoint;
};
struct BearingOnlyVars{
    bool only_direction_mode;
    bool have_to_avoid_obstacle;
};


/**
 * Collision avoidance class
 */
class CollisionAvoidanceNode : public Node{
    //For test only
public:
    CollisionAvoidanceNode(MessageBus& msgBus);
    ~CollisionAvoidanceNode() {};

    /**
     * Nothing to initialize for now
     * @return
     */
    bool init();

    /**
     * Process the messages from the message bus.
     * Called by message bus.
     * @param message
     * @return
     */
    void processMessage(const Message* message);

    /**
     * Setter for the sailing zone
     * @return
     */
    bool setSailingZone();

protected:

    std::vector<Obstacle> m_seenObstacles;
    FollowedLine m_followedLine;
    std::vector<Eigen::Vector2d> m_sailingZone;
    SensorData m_sensorOutput;
    bool m_tack; //Need init
    int m_tackingDirection; //Need init
    double m_loop_id;
    BearingOnlyVars m_bearingOnlyVars; //For future implementation if collision avoidance doesn't work

    //PRIVATE MAIN FUNCTIONS

    /**
     * Process the message that constains the state of the boat
     * @param msg
     */
    void processVesselState(VesselStateMsg* msg);

    /**
     * Proccess the messages from the color_detection node
     * @param msg
     */
    void processObstacleData(ObstacleData* msg);

    /**
     * Process the messages from the waypoint manager in order to know the
     * line the boat is currently following.
     * @param msg
     */
    void processWaypointData(WaypointDataMsg* msg);

    /**
     * The most important function of the class, it calls al the others.
     * It's called by processObstacleData.
     *
     * It uses m_followedLine and m_sensorOutput as main input
     * Then it sends the commands by itself
     */
    void run();

    /**
     * Is there any obstacles ? If yes, which information can i gather on them.
     * OUTPUT a list of obstacles (struct) with all their characteristics.
     *
     * For now transform hardcoded obstacles to something more usable.
     * @param sensorData
     * @return
     */
    void check_obstacles(SensorData sensorData,
                         std::vector<Obstacle> &seenObstacles);

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
    WaypointOutput compute_new_path(Eigen::Vector2d collision_avoidance_point,
                                    FollowedLine followedLine);

    /**
     * Compute the commands according to the new path.
     * Follow the line between the last waypoint and the next
     * @param line
     * @return
     */
    CommandOutput compute_commands(FollowedLine line);

    // UTILITY FUNCTIONS
    /*
     * Most of them are functions to handle geometry.
     */

    /**
     * Draw the state of the boat on vibes.
     */
    void drawState();
    std::vector<double> getVectorLine(std::vector<Eigen::Vector2d> vec,int line);
    void drawObstacles(std::vector<Obstacle> seen_obstacles,std::string color);
    void drawObstacle(Obstacle obs,std::string color);
    void drawEigenPoly(std::vector<Eigen::Vector2d> poly,std::string color);
    void drawBoat(SensorData sensorData,std::string color);
    void drawChannel(FollowedLine followedLine);
    void drawPotField(PotentialMap potfield,int option);
    void drawPotFieldPoint(int i, int j,
                                                   PotentialMap potfield,
                                                   std::string color, int option);

    /**
     * Gives the sum between two angles regardless of their definition.
     * It's radAngle1+radAngle2.
     * The angles needs to be in radians.
     * @param radAngle1
     * @param radAngle2
     * @return
     */
    double wrapToPi(double radAngle1,
                    double radAngle2);

    /**
     * \Brief Compute the distance between 2 GPS points
     * Haversine algorithm for distance computation on Earth. \n
     * Took on http://www.movable-type.co.uk/scripts/latlong.html \n
     * a = sin²(Δφ/2) + cos φ1 ⋅ cos φ2 ⋅ sin²(Δλ/2) \n
     * c = 2 ⋅ atan2( √a, √(1−a) ) \n
     * distance = Rearth ⋅ c \n
     * @return double          The distance between the two gps points
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
     *
     * It's not used for now but kept in case of future implementation
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
     * Here everything is in meters and radians
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
     * recreate the polygon but not the obstacle
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
    void cleanObstacles(SensorData sensorData,
                        std::vector<Obstacle> &seenObstacles);

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
    void mergeObstacles(SensorData sensorData,
                        std::vector<Obstacle> upToDateObstacles,
                        std::vector<Obstacle> &seenObstacles);

    /**
     * Boost union handler
     * @param mpoly
     * @return
     */
    boostMultiPolygon polyUnion(boostMultiPolygon mpoly);

    /**
     * Compute obstacle potential
     *
     * Since the obstacle is a polygon, only the closest point from
     * the polygon will be counted as an obstacle
     *
     * Maybe we should do a parameter file where we could configure everything
     * Values found after extensive testing on matlab
     * The values are scaled for meters.
     * @param Px
     * @param Py
     * @param sensorData
     * @param seenObstacles
     * @param followedLine
     * @return
     */
    Eigen::ArrayXXd computeObstaclePot(Eigen::ArrayXXd Px,Eigen::ArrayXXd Py,
                                       SensorData sensorData,std::vector<Obstacle> seenObstacles,
                                       FollowedLine followedLine);
    /**
     * Compute objective potential
     * @param Px  a grid matrix used like an axis
     * @param Py  a grid matrix used like an axis
     * @param followedLine
     * @return
     */
    Eigen::ArrayXXd computeObjectivePot(Eigen::ArrayXXd Px,Eigen::ArrayXXd Py,FollowedLine followedLine);
    /**
     * Compute wind no-go zone potential
     * @param Px
     * @param Py
     * @param sensorData
     * @return
     */
    Eigen::ArrayXXd computeWindPot(Eigen::ArrayXXd Px,Eigen::ArrayXXd Py,SensorData sensorData);
    /**
     * Compute boat potential
     * @param Px
     * @param Py
     * @param sensorData
     * @return
     */
    Eigen::ArrayXXd computeBoatPot(Eigen::ArrayXXd Px,Eigen::ArrayXXd Py,SensorData sensorData);
    /**
     * Compute sailing zone potential
     * @param Px
     * @param Py
     * @param sailingZone
     * @return
     */
    Eigen::ArrayXXd computeSailingZonePot(Eigen::ArrayXXd Px,Eigen::ArrayXXd Py,
                                          std::vector<Eigen::Vector2d> sailingZone);


};

#endif //SAILINGROBOT_TEST_AVOIDANCE_BEHAVIOUR_H