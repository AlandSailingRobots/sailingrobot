//
// Created by tag on 12/07/16.
//

#ifndef SAILINGROBOT_TEST_AVOIDANCE_BEHAVIOUR_H
#define SAILINGROBOT_TEST_AVOIDANCE_BEHAVIOUR_H

#endif //SAILINGROBOT_TEST_AVOIDANCE_BEHAVIOUR_H

#include <vector>
#include <math.h>
#include <stdlib.h>
#include "libs/Eigen/Dense"
#include "RoutingBehaviour.h"

// TODO : Receive these values from the database
#define DISTANCE_NOT_THE_SAME_OBSTACLE 2.0
#define MAXIMUM_SENSOR_RANGE 100.0
#define SENSOR_HEADING_RELATIVE_TO_BOAT 0.0 // There might be several sensors
#define SENSOR_ARC_ANGLE 90.0 // Every angle is in radian
#define CHANNEL_WIDTH 15.0
#define SAFE_DISTANCE 30.0
#define EARTH_RADIUS 6371000

// SAVED CODE IN CASE OF ARCHITECTURAL CHANGE
/*

//struct sensorData{
//    double gpsLon; //x
//    double gpsLat; //y
//    double compass;
//    double windDirection;
//};

*/

/**
 * Collision avoidance class
 * Super class call. Calls the database
 */
class CollisionAvoidanceBehaviour : public RoutingBehaviour {
    CollisionAvoidanceBehaviour(DBHandler *db);

    ~CollisionAvoidanceBehaviour() { };

public:
    /**
     * Nothing to init for now
     * @return
     */
    bool init(); //

    /**
     * Compute the commands
     * Too much output parameters, should be reduced.
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
    //make the computation of the commands

private:

    std::vector<Obstacle> seenObstacles;
    FollowedLine followedLine;
    std::vector<Eigen::Vector2d> sailingZone;
    SensorData sensorOutput;

    /**
     * Structure which contains every used data from the sensors
     */
    struct SensorData {
        Eigen::Vector2d gpsPos;
        double gpsSpeed;
        double compHeading;
        double gpsHeading;
        double windDirection;
        double windSpeed;
        int pitch;
        int roll;

        //TODO : put the real inputs here (video, sonar or whatever)
        std::vector<ObstacleData> detectedObstacles;
    };
    struct ObstacleData {
        double minDistanceToObstacle;
        double maxDistanceToObstacle; /**< -1 = infinite */
        double LeftBoundHeadingRelativeToBoat;
        double RightBoundHeadingRelativeToBoat;
    };
    struct Obstacle {
        //Polygon
        std::vector<Eigen::Vector2d> polygon;
        std::string color;
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
    };
    const struct Simulation {
        const bool waypoints;
        const bool obstacles;
    };

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
     * Haversine algorithm for distance computation on Earth.
     * Took on http://www.movable-type.co.uk/scripts/latlong.html
     * a = sin²(Δφ/2) + cos φ1 ⋅ cos φ2 ⋅ sin²(Δλ/2)
     * c = 2 ⋅ atan2( √a, √(1−a) )
     * distance = Rearth ⋅ c
     * @return
     */
    double calculateGPSDistance(Eigen::Vector2d, Eigen::Vector2d);

    /**
     * Find the center of a polygon defined anticlockwise.
     * @param polygon
     * @return
     */
    Eigen::Vector2d findCenter(const std::vector<Eigen::Vector2d> polygon);

    /**
     * Gives the area in unit of the vector (squared of course).
     * So be careful of the units of the values inside the vectors
     * The polygon must be initialized anticlockwise
     * @param polygon
     * @return
     */
    double getArea(std::vector<Eigen::Vector2d> polygon);

    /**
     * Compute the signed distance between a line and a point
     * Uses eigen::vector2d in GPS coordinates
     * @param linePt1
     * @param linePt2
     * @param point
     * @return
     */
    double signedDistanceFromLine(Eigen::Vector2d linePt1,Eigen::Vector2d linePt2,Eigen::Vector2d point);

    /**
     * Get the closest point between the given polygon and a point.
     * Got from http://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment
     * and implemented it for this code.
     *
     * the coordinates of the points of the polygon are in latitude and longitude
     * @param polygon
     * @param point
     * @return
     */
    Eigen::Vector2d getClosestPoint(std::vector<Eigen::Vector2d> polygon, Eigen::Vector2d point);

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
     * Distance from a segment on a sphere (the earth)
     * Got from Robotic Sailing 2012 by Colin Sauzé and James Finnis
     *
     *  Uses getClosestPoint
     * @param segmentPoint1
     * @param segmentPoint2
     * @param point
     * @return
     */
    double distanceFromSegment(Eigen::Vector2d segmentPoint1,Eigen::Vector2d segmentPoint2,Eigen::Vector2d point);

    /**
     * Verify if the projection of a point on a plane is inside a trigle of the plane
     * Got from
     * http://math.stackexchange.com/questions/544946/determine-if-projection-of-3d-point-onto-plane-is-within-a-triangle
     *
     * Every polygon in this program is defined anti-clockwise
     * @param triangle
     * @param point
     * @return
     */
    bool projectionInside3DTriangle(Eigen::Vector3d triangle[3],Eigen::Vector3d point);

    /**
     * Debugging function, specific to Eigen
     * @param name
     * @param v
     */
    void printStdVectorMat(std::string const &name,
                           std::vector<Eigen::MatrixXd> const &v);

    /**
     * Debugging functions, specific to Eigen
     * @param name
     * @param v
     */
    void printStdVectorFloat(std::string const &name,
                             std::vector<float> const &v);

    /**
     * Debugging functions, specific to Eigen
     * @param name
     * @param mat
     */
    void printMat(std::string const &name,
                  Eigen::MatrixXd const &mat);

    SensorData update_sensors(SystemStateModel &systemStateModel,
                              Simulation sim);

    std::vector<Obstacle> check_obstacles(SensorData sensorData);

    bool these_obstacles_are_a_problem(std::vector<Obstacle> seenObstacles);

    Eigen::MatrixXd compute_potential_field(std::vector<Obstacle> seen_obstacles,
                                            std::vector<Eigen::Vector2d> sailing_zone,
                                            FollowedLine followedLine);

    MinPotField find_minimum_potential_field(Eigen::MatrixXd Potential_field);

    FollowedLine compute_new_path(MinPotField min);

    CommandOutput compute_commands(FollowedLine line);

    CommandOutput run(SystemStateModel &systemStateModel);
};
