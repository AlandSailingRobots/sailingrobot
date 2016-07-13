//
// Created by Simon CHANU on 11/07/16.
//

#include "CollisionAvoidanceBehaviour.h"

//INPUT
/* Current_line (WP1 and WP2)
 * Boat_state
 *     GPS position
 *     roll, pitch, yaw
 *     speed
 * Sensors_output
 * sailing_zone (gps points)
 * wind direction and strength
 */
//IDEA
/* To use dynamic list of struct for obstacles and other variables such as this one
 */
//STRUCTS
/*  map{obstacles, sailing_zone, followed_line}
 *  obstacle{x, y, direction, interval of distance, interval of bearing, color}
 */
//I WANT TO TRACK
/* boat position and heading
 * the obstacles with everything
 */
// SAVED CODE IN CASE OF ARCHITECTURAL CHANGE
/*

//sensorData update_sensor(){}

//    commandOutput run(INPUT){
//        sensors output = update_sensors(); //=> gives sensors output or compute an easier way to handle them
//        update_waypoints(); //=> update waypoints or compute an easier way to handle them
//        check_obstacles();
//    //    update_map();
//        if(these_obstacles_are_a_problem()){
//            Eigen::MatrixXd potential_field = compute_potential_field();
//            minPotField min = find_minimum_potential_field(potential_field);
//            compute_new_path();
//        }
//        return compute_commands();
//    }


*/

/*
 * Makes the interface between the old code and the new one. This is make the code more modular
 * since changes in the architecture might come.
 */
SensorData CollisionAvoidanceBehaviour::update_sensor(SystemStateModel &systemStateModel, bool simulation) {
    //Extraction of data
    SensorData sensorData;

    //Position and speed
    sensorData.gpsLat = systemStateModel.gpsModel.positionModel.latitude;
    sensorData.gpsLon = systemStateModel.gpsModel.positionModel.longitude;
    sensorData.gpsSpeed = systemStateModel.gpsModel.speed;

    //Heading
    //compHeading : degree from north -> radian from east
    sensorData.compHeading = Utility::degreeToRadian(systemStateModel.compassModel.heading)
                             - M_PI / 2;
    //gpsHeading : degree from north -> radian from east
    sensorData.gpsHeading = Utility::degreeToRadian(systemStateModel.gpsModel.heading)
                            - M_PI / 2;

    //Wind
    //windDirection : degree from north -> radian from east
    sensorData.windDirection = Utility::degreeToRadian(systemStateModel.windsensorModel.direction)
                               - M_PI / 2;
    sensorData.windSpeed = systemStateModel.windsensorModel.speed;

    //Tilt
    sensorData.pitch = systemStateModel.compassModel.pitch;
    sensorData.roll = systemStateModel.compassModel.roll;
    return sensorData;
}

//    void update_waypoints(){}
/*
 * Is there any obstacles ? If yes, which information can i gather on them.
 */
std::vector<Obstacle> CollisionAvoidanceBehaviour::check_obstacles(SensorData sensorData) { // OUTPUT a list of obstacles (struct)
    //Is there any obstacles ?

}
//map update_map(){}
/*
 * Check if there is intersection between the current path+security radius and the obstacle
 */
bool CollisionAvoidanceBehaviour::these_obstacles_are_a_problem(
        std::vector<Obstacle> seenObstacles) { // OUTPUT if these obstacles are a problem

}

/*
 *
 */
Eigen::MatrixXd CollisionAvoidanceBehaviour::compute_potential_field(
        std::vector<Obstacle> seen_obstacles, sailing_zone, FollowedLine) {

}

/*
 *
 */
MinPotField CollisionAvoidanceBehaviour::find_minimum_potential_field(Potential_field) {

}

/*
 * Gives the new line to follow. It would be better if it added a WP in the DataBase as well.
 */
CommandOutput CollisionAvoidanceBehaviour::compute_new_path() {

}

/*
 * Main function of the class. Should be replaced by run as soon as possible.
 */
void CollisionAvoidanceBehaviour::computeCommands(
        SystemStateModel &systemStateModel, std::unique_ptr<Position> const &position,
        double trueWindDirection, bool mockPosition, bool getHeadingFromCompass) {

    sensorsOutput = update_sensors(SystemStateModel & systemStateModel, bool
    simulation); //=> gives sensors output or compute an easier way to handle them

    //update_waypoints(); //=> update waypoints or compute an easier way to handle them

    seenObstacles = check_obstacles(sensorsOutput);
    //    update_map();
    if (these_obstacles_are_a_problem()) {
        Eigen::MatrixXd potential_field = compute_potential_field();
        minPotField min = find_minimum_potential_field(potential_field);
        compute_new_path();
    }
    return compute_commands();
}
