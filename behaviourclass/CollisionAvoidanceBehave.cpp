#include "CollisionAvoidanceBehave.h"
CollisionAvoidanceBehave::CollisionAvoidanceBehave(DBHandler *db):
  RoutingBehaviour(db)//super class call
{
    potential_field_dim(1,4);//Matlab name: dim //xmin xmax ymin ymax
    potential_field_dim(0,1)=-100;
    potential_field_dim(0,2)=100;
    potential_field_dim(0,3)=-100;
    potential_field_dim(0,4)=100;
    int step=5;
    int rows_Z = (int)(abs(potential_field_dim(0,3)-potential_field_dim(0,4))/step)+1;
    int cols_Z = (int)(abs(potential_field_dim(0,3)-potential_field_dim(0,4))/step)+1;
    potential_Z(rows_Z,cols_Z);//Matlab name: Z
    Eigen::VectorXd::LinSpaced(cols_Z,potential_field_dim(0,1),potential_field_dim(0,2));
    /*
    Eigen::MatrixXd point_x;//Matlab name: P1
    Eigen::MatrixXd point_y;//Matlab name: P2
    float step_coeff;//Matlab name: eta //In direction mode length of the repulsion of the obstacle


    //Obstacles
    std::vector<Eigen::MatrixXd> mock_obstacle_list;//Matlab name: posObstacles //position qnd speed of mock obstacles
    std::vector<Eigen::MatrixXd> detected_obstacle_list_qhat;//Matlab name:  qhat
    std::vector<Eigen::MatrixXd> detected_obstacles;//Matlab name: detectedObstacles
    std::vector<Eigen::MatrixXd> collisioned_obstacle;//Matlab name: collisionedObstacle//Incle a memory of the previous encountered obstacles if not in direction mode
    float direction_boat_obstacle;//Matlab name: directionObstacleDetected
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
*/
}

bool CollisionAvoidanceBehave::init()
{

    return true;
}

void CollisionAvoidanceBehave::computeCommands(SystemStateModel &systemStateModel,std::unique_ptr<Position> const& position,
                                    double trueWindDirection, bool mockPosition,
                                    bool getHeadingFromCompass) {}

void CollisionAvoidanceBehave::manageDatabase(double trueWindDirection, SystemStateModel &systemStateModel){}
void CollisionAvoidanceBehave::setupWaypoints(){}
void CollisionAvoidanceBehave::setNextWaypoint(){}
void CollisionAvoidanceBehave::setPreviousWayPoint(SystemStateModel &m_systemStateModel){}
int CollisionAvoidanceBehave::getHeading(SystemStateModel &m_systemStateModel,bool m_mockPosition,bool m_getHeadingFromCompass,std::unique_ptr<Position> const& position) {
    return 1;
}

void CollisionAvoidanceBehave::mockObstacleDetection(){}//Matlab name: calculate_potField
void CollisionAvoidanceBehave::calculatePotentialField(){}//Matlab name: calculate_potField
void CollisionAvoidanceBehave::avoidObstacle(){}//Matlab name: avoid_obstacle
void CollisionAvoidanceBehave::obstacleOnACollisionCourse(){}//Matlab name: boat_on_collision_course
void CollisionAvoidanceBehave::calculate_collision_avoidance_point(){}//Matlab name: calculate_avoidCollisionPoint
Eigen::MatrixXd CollisionAvoidanceBehave::createWall(Eigen::MatrixXd const& starting_point,Eigen::MatrixXd const& ending_point,float step){
    Eigen::MatrixXd m(2,2);
    m(0,0) = 3;
    m(1,0) = 2.5;
    m(0,1) = -1;
    m(1,1) = m(1,0) + m(0,1);
    return m;
}
void CollisionAvoidanceBehave::moveObstacle(std::vector<Eigen::MatrixXd>& mock_obstacle_list,std::vector<int> elements, float dt){}
