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

// SAVED CODE IN CASE OF ARCHITECTURAL CHANGE
/*

//struct sensorData{
//    double gpsLon; //x
//    double gpsLat; //y
//    double compass;
//    double windDirection;
//};

*/

class CollisionAvoidanceBehaviour : public RoutingBehaviour {
    CollisionAvoidanceBehaviour(DBHandler *db);

    ~CollisionAvoidanceBehaviour() { };

public:
    bool init(); //Nothing to init for now

    void computeCommands(SystemStateModel &systemStateModel,
                         std::unique_ptr<Position> const &position,
                         double trueWindDirection,
                         bool mockPosition,
                         bool getHeadingFromCompass);
    //make the computation of the commands

private:
    //Boate state vector
    double gpsBoatLon;

    std::vector<Obstacle> seenObstacles;
    FollowedLine followedLine;
    std::vector<Eigen::Vector2d> sailingZone;
    SensorData sensorOutput;

    // For now this is hardcoded but it should be in the database.

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
        double maxDistanceToObstacle; // -1 = infinite
        double LeftBoundHeadingRelativeToBoat;
        double RightBoundHeadingRelativeToBoat;
    };
    struct Obstacle {
        //Polygon
        std::vector<Eigen::Vector2d> polygon;
//        double leftBoundHeading;
//        double rightBoundHeading;
//        double upperBoundDistance;
//        double lowerBoundDistance;
//        double xGPSBoatPositionAtDetection;
//        double yGPSBoatPositionAtDetection;
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

    double angleDiff(double radAngle1,
                     double radAngle2);

    double calculateGPSDistance(Eigen::Vector2d, Eigen::Vector2d);

    Eigen::Vector2d findCenter(const std::vector<Eigen::Vector2d> polygon);

    double getArea(std::vector<Eigen::Vector2d> polygon);

    double computeSignedDistanceFromLine(Eigen::Vector2d linePt1,Eigen::Vector2d linePt2,Eigen::Vector2d point);

    void printStdVectorMat(std::string const &name,
                           std::vector<Eigen::MatrixXd> const &v);

    void printStdVectorFloat(std::string const &name,
                             std::vector<float> const &v);

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
