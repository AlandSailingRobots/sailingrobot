#ifndef __COLLISIONAVOIDANCEBEHAVE_H__
#define __COLLISIONAVOIDANCEBEHAVE_H__
#include <stdint.h> //uint8_t
#include <math.h>
#include <algorithm>
#include <math.h>
#include <cmath>
#include <iostream>
#include "coursecalculation/CourseCalculation.h"
#include "RoutingBehaviour.h"
#include "utility/Utility.h"
#include "waypointrouting/WaypointRouting.h"
#include "behaviourclass/LineFollowBehaviour.h"
#include "libs/Eigen/Dense"
#include <vector>

class CollisionAvoidanceBehave:public RoutingBehaviour{
public:
    CollisionAvoidanceBehave(DBHandler *db);
    ~CollisionAvoidanceBehave(){};

    bool init();
    void computeCommands(SystemStateModel &systemStateModel,std::unique_ptr<Position> const& position,
                                        double trueWindDirection, bool mockPosition,
                                        bool getHeadingFromCompass);
    void manageDatabase(double trueWindDirection, SystemStateModel &systemStateModel);
/*
    double getRudderCommand();
    double getSailCommand();
*/
private:

    void setupWaypoints();
    void setNextWaypoint();
    void setPreviousWayPoint(SystemStateModel &m_systemStateModel);
    int getHeading(SystemStateModel &m_systemStateModel,bool m_mockPosition,bool m_getHeadingFromCompass,std::unique_ptr<Position> const& position);
    //Potential Field
    Eigen::MatrixXd potential_Z;//Matlab name: Z
    Eigen::MatrixXd point_x;//Matlab name: P1
    Eigen::MatrixXd point_y;//Matlab name: P2
    float step_coeff;//Matlab name: eta //In direction mode length of the repulsion of the obstacle
    Eigen::MatrixXd potential_field_dim; //Matlab name: dim //xmin xmax ymin ymax

    //Obstacles
    std::vector<Eigen::MatrixXd> mock_obstacle_list;//Matlab name: posObstacles //position qnd speed of mock obstacles
    std::vector<Eigen::MatrixXd> detected_obstacle_list_qhat;//Matlab name:  qhat
    std::vector<Eigen::MatrixXd> detected_obstacles;//Matlab name: detectedObstacles
    std::vector<Eigen::MatrixXd> collisioned_obstacle;//Matlab name: collisionedObstacle//Incle a memory of the previous encountered obstacles if not in direction mode
    std::vector<float> direction_boat_obstacle;//Matlab name: directionObstacleDetected //Empty when no direction
    float radius_obstacle;//Matlab name: rq  //security radius around the obstacle
    Eigen::MatrixXd collision_avoidance_point;//Matlab name: avoidCollisionPoint // Point to follow when an obstacle is detected
    int is_obstacle_detected;//Matlab name: is_obstacle_detected //1 if an obstacle is detected

    //Targets
    std::vector<Eigen::MatrixXd> target_list;//Matlab name: posWaypoints
    Eigen::MatrixXd target_phat;//Matlab name: phat

    //Boat
    Eigen::MatrixXd boat_state;//Matlab name: x
    float mock_detection_distance;//Matlab name: distDetect//Range of detection of obstacles
    float mock_detection_angle;//Matlab name: angleDetect//Angle of detection of obstacles
    std::vector<Eigen::MatrixXd> line_to_follow;//Matlab name: followedLine
    float radius_corridor;//Matlab name: r//corridor to stay in during line following


    //World
    float wind_direction;//Matlab name: psi

    //Different mode
    int only_direction_mode;//Matlab name:  onlyHeadingMode //Do the boat know the position or the direction of the obstacle?
    int have_to_avoid_obstacle;//Matlab name: haveToAvoidObstacle //In direction mode do the boat need to avoid an obstacle?
    int can_compute_a_new_avoidance_point;//Matlab name: avoidMode //Can the boat compute a new collision_avoidance_point

    /*
    mockObstacleDetection() : fake the detection of an obstacle according to an mock_detection_angle and a mock_detection_distance
    Uses:
        MatrixXd boat_state,
        std::vector<Eigen::MatrixXd> mock_obstacle_list
        float mock_detection_distance,
        float mock_detection_angle,
    Modified attributes:
        std::vector<Eigen::MatrixXd> detected_obstacles,
        float direction_boat_obstacle
    */
    void mockObstacleDetection();//Matlab name: obstacle_detection

/*
    calculatePotentialField : Potential field computation to place a collision_avoidance_point in its global minimum
    Uses:
        MatrixXd point_x,
        MatrixXd point_y,
        MatrixXd boat_state,
        MatrixXd target_phat,
        std::vector<Eigen::MatrixXd> detected_obstacle_list_qhat,
        float wind_direction,
        int only_direction_mode,
        int have_to_avoid_obstacle,
        float step_coeff,
        float direction_boat_obstacle,
    Modified attributes:
        MatrixXd potential_Z
*/
    void calculatePotentialField();//Matlab name: calculate_potField
/*
    avoidObstacle : Do the computation to avoid the obstacle
    Uses:
        MatrixXd target_phat,
        MatrixXd boat_state,
        MatrixXd point_x,
        MatrixXd point_y,
        float wind_direction,
        float radius_obstacle,
        float radius_corridor,
        float dim,
        int only_direction_mode,
        int have_to_avoid_obstacle,
        float step_coeff,
        float direction_boat_obstacle
    Modified attributes:
        int can_compute_a_new_avoidance_point,
        std::vector<Eigen::MatrixXd> line_to_follow,
        MatrixXd const collisioned_Obstacle,
        MatrixXd avoidance_collision_point,
        std::vector<Eigen::MatrixXd> detected_obstacle_list_qhat,

*/
    void avoidObstacle();//Matlab name: avoid_obstacle
/*
    obstacleOnACollisionCourse: check if an object is on the boat course. Take into account a memory of the already encountered obstacles:
    Uses:
        MatrixXdboat_state,
        std::vector<Eigen::MatrixXd> detected_obstacle_list_qhat,
        float radius_obstacle,
        int can_compute_a_new_avoidance_point,
        std::vector<MatrixXd> detected_obstacles,
    Modified attributes:
        MatrixXd collisioned_Obstacle,
        int only_direction_mode,
        int is_obstacle_detected;
        int have_to_avoid_obstacle;

*/
    void obstacleOnACollisionCourse();//Matlab name: boat_on_collision_course
    /*
    calculate_collision_avoidance_point: compute the global minimu of the Z field and return it
    Uses:
        MatrixXd potential_Z;
        MatrixXd potential_field_dim;
    Modified attributes:
        std::vector<MatrixXd> collisioned_obstacle;
    */

    void calculate_collision_avoidance_point();//Matlab name: calculate_avoidCollisionPoint

    /*
    calculate_collision_avoidance_point: compute the global minimu of the Z field and return it
    Uses:
        std::vector<Eigen::MatrixXd> detected_obstacles,
        MatrixXd boat_state,
        float mock_detection_distance,
        float mock_detection_angle,
    Modified attributes:
        std::vector<Eigen::MatrixXd> detected_obstacle_list_qhat,
    */

    void update_obstacles();//MATLAB name : update_obstacles


    Eigen::MatrixXd createWall(Eigen::MatrixXd const& starting_point,Eigen::MatrixXd const& ending_point,float step);

    void moveObstacle(std::vector<Eigen::MatrixXd>& mock_obstacle_list,std::vector<int> elements, float dt);


    Eigen::MatrixXd atanMat(Eigen::MatrixXd mat);
    float heaviside(float num);
    Eigen::MatrixXd heavisideMat(Eigen::MatrixXd mat);
    Eigen::MatrixXd rectangularPulse(float a, float b, Eigen::MatrixXd x);
    void printStdVectorMat(std::string const& name, std::vector<Eigen::MatrixXd> const& v);
    void printMat(std::string const& name,Eigen::MatrixXd const& mat);
    void printStdVectorFloat(std::string const& name, std::vector<float> const& v);
    void printCollisionAvoidanceBehave(int rows_Z,int cols_Z,Eigen::VectorXd v);


};

#endif
