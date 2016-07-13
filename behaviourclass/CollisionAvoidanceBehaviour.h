//
// Created by tag on 12/07/16.
//

#ifndef SAILINGROBOT_TEST_AVOIDANCE_BEHAVIOUR_H
#define SAILINGROBOT_TEST_AVOIDANCE_BEHAVIOUR_H

#endif //SAILINGROBOT_TEST_AVOIDANCE_BEHAVIOUR_H

#include <vector>
#include <math.h>
#include "libs/Eigen/Dense"
#include "RoutingBehaviour.h"

// SAVED CODE IN CASE OF ARCHITECTURAL CHANGE
/*

//struct sensorData{
//    double gpsLon; //x
//    double gpsLat; //y
//    double compass;
//    double windDirection;
//};

*/

class CollisionAvoidanceBehaviour:public RoutingBehaviour{
    CollisionAvoidanceBehaviour(DBHandler *db);
    ~CollisionAvoidanceBehaviour(){};

public:
    bool init(); //Nothing to init for now

    void computeCommands(
            SystemStateModel &systemStateModel,std::unique_ptr<Position> const& position,
            double trueWindDirection, bool mockPosition, bool getHeadingFromCompass);
            //make the computation of the commands return true if the computation was successfull






private:
    std::vector<Obstacle> seenObstacles;
    FollowedLine followedLine;
    std::vector<Eigen::Vector2d> sailingZone;

    struct SensorData{
        double gpsLon; //x
        double gpsLat; //y
        double gpsSpeed;
        double compHeading;
        double gpsHeading;
        double windDirection;
        double windSpeed;
        int pitch;
        int roll;
    };
    struct Obstacle{
        double x;
        double y;
        double direction;
        double directionInterval;
        double distanceInterval;
        std::string color;
    };
    struct FollowedLine{
        Eigen::Vector2d startPoint;
        Eigen::Vector2d endPoint;
    };
    struct CommandOutput{
        double deltaRudder;
        double deltaSail;
    };
    struct MinPotField{
        double x;
        double y;
        double row;
        double col;
    };

    SensorData update_sensors(SystemStateModel &systemStateModel, bool simulation);
    std::vector<Obstacle> check_obstacles(SensorData sensorData);
    bool these_obstacles_are_a_problem(
            std::vector<Obstacle> seenObstacles);
    Eigen::MatrixXd compute_potential_field(
            std::vector<Obstacle> seen_obstacles, std::vector<Eigen::Vector2d> sailing_zone, FollowedLine);
    MinPotField find_minimum_potential_field(Eigen::MatrixXd Potential_field);
    CommandOutput compute_new_path();
};
