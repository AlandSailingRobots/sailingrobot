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

//UTILITY FUNCTIONS
/*
 * Might be put into Utility class soon. Need to check that.
 */
double angleDiff(double angle1, double angle2){

}
void CollisionAvoidanceBehaviour::printStdVectorMat(std::string const& name, std::vector<Eigen::MatrixXd> const& v){

    std::cout << " " << std::endl;
    std::cout << name << " : " << std::endl;
    std::cout << " " << std::endl;
    for(int i=0; i<(int)v.size(); ++i){
        std::cout <<v[i] << std::endl;
        std::cout << " " << std::endl;
    }
}
void CollisionAvoidanceBehaviour::printStdVectorFloat(std::string const& name, std::vector<float> const& v){

    std::cout << " " << std::endl;
    std::cout << name << " : " << std::endl;
    std::cout << " " << std::endl;
    for(int i=0; i<(int)v.size(); ++i){
        std::cout <<v[i] << std::endl;
        std::cout << " " << std::endl;
    }
}
void CollisionAvoidanceBehaviour::printMat(std::string const& name,Eigen::MatrixXd const& mat){

    std::cout << " " << std::endl;
    std::cout << name << " : " << std::endl;
    std::cout << " " << std::endl;
    std::cout << mat<< std::endl;
    std::cout << " " << std::endl;
}

//MAIN FUNCTIONS
/*
 * Makes the interface between the old code and the new one. This is make the code more modular
 * since changes in the architecture might come.
 */
SensorData CollisionAvoidanceBehaviour::update_sensors(SystemStateModel &systemStateModel, bool simulation) {
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
 * Compute the potential field for the obstacles, the sailing zone,
 * the boat, the objective, and the wind.
 *
 * For now it creates the matrix at each loop (easier for code review)
 * The size of the matrix needs to be adapted to the size of the sailing zone
 * (with the max and mins for example)
 */
Eigen::MatrixXd CollisionAvoidanceBehaviour::compute_potential_field(
        std::vector<Obstacle> seen_obstacles, sailing_zone, FollowedLine) {

}

/*
 * Find the minimum in the potential field and return its coordinates in the matrix
 * as well as its real gps coordinates.
 */
MinPotField CollisionAvoidanceBehaviour::find_minimum_potential_field(Eigen::MatrixXd Potential_field) {

}

/*
 * Gives the new line to follow. It would be better if it added a WP in the DataBase as well.
 */
CommandOutput CollisionAvoidanceBehaviour::compute_new_path() {

}

/*
 * The most important function of the class, it calls al the others.
 * Should be replaced by run as soon as possible.
 */
void CollisionAvoidanceBehaviour::computeCommands(
        SystemStateModel &systemStateModel, std::unique_ptr<Position> const &position,
        double trueWindDirection, bool mockPosition, bool getHeadingFromCompass) {

    //Note
    /*
     * For now i don't know of any place in the code where it specified if the code is
     * in a simulated environement or not. So this variable is temporary.
     */
    bool simulation = 0;

    //Gives sensors output or compute an easier way to handle them
    SensorData sensOutput = CollisionAvoidanceBehaviour::update_sensors(&systemStateModel, simulation);

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
