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
#define CONVERSION_FACTOR_METER_TO_GPS 0.00000899280575539

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
     * Initialize values :
     *  sailingZone
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
    //make the computation of the commands

private:

    /**
     * Structure which contains every used data from the sensors
     */
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
        const bool waypoints;
        const bool obstacles;
    };
    struct PotentialMap{
        const double xMin;
        const double xMax;
        const double yMin;
        const double yMax;
        Eigen::ArrayXXd field;
    };

    std::vector<Obstacle> seenObstacles;
    FollowedLine followedLine;
    std::vector<Eigen::Vector2d> sailingZone;
    SensorData sensorOutput;

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
    bool projectionInsideSlice(Eigen::Vector3d triangle[3],Eigen::Vector3d point);

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
    std::vector<Obstacle> check_obstacles(SensorData sensorData);

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
