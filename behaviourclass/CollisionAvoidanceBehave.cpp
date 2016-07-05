#include "CollisionAvoidanceBehave.h"
CollisionAvoidanceBehave::CollisionAvoidanceBehave(DBHandler *db):
  RoutingBehaviour(db)//super class call
{
    potential_field_dim = Eigen::MatrixXd::Zero(1,4);//Matlab name: dim //xmin xmax ymin ymax
    potential_field_dim(0,0)=-100;
    potential_field_dim(0,1)=100;
    potential_field_dim(0,2)=-100;
    potential_field_dim(0,3)=100;
    std::cout << " " << std::endl;
    std::cout << "potential_field_dim: "<< std::endl;
    std::cout << " " << std::endl;
    std::cout << potential_field_dim<< std::endl;
    std::cout << " " << std::endl;
    int step=5;
    int rows_Z = (int)(abs(potential_field_dim(0,2)-potential_field_dim(0,3))/step)+1;
    int cols_Z = (int)(abs(potential_field_dim(0,2)-potential_field_dim(0,3))/step)+1;
    potential_Z = Eigen::MatrixXd::Zero(rows_Z,cols_Z);//Matlab name: Z
    Eigen::VectorXd v = Eigen::VectorXd::LinSpaced(cols_Z,potential_field_dim(0,0),potential_field_dim(0,1));

    std::cout << "rows_Z: "<< rows_Z << std::endl;
    std::cout << " " << std::endl;
    std::cout << "cols_Z: "<< cols_Z << std::endl;
    std::cout << " " << std::endl;
    std::cout << "potential_Z: "<< std::endl;
    std::cout << " " << std::endl;
    std::cout << potential_Z<< std::endl;
    std::cout << " " << std::endl;
    std::cout << "v "<< v.adjoint() << std::endl;

    point_x = Eigen::MatrixXd::Zero(rows_Z,cols_Z);//Matlab name: P1
    point_y = Eigen::MatrixXd::Zero(rows_Z,cols_Z);//Matlab name: P2

    for(int i=0; i<rows_Z; i++){
        point_x.row(i)=v.adjoint();
        point_y.col(i)=v;
    }
    std::cout << " " << std::endl;
    std::cout << "point_x: "<< std::endl;
    std::cout << " " << std::endl;
    std::cout << point_x<< std::endl;
    std::cout << " " << std::endl;
    std::cout << "point_y: "<< std::endl;
    std::cout << " " << std::endl;
    std::cout << point_y<< std::endl;

    std::cout << " " << std::endl;
    step_coeff=3;//Matlab name: eta //In direction mode length of the repulsion of the obstacle
    std::cout << "step_coeff : " << step_coeff << std::endl;
    std::cout << " " << std::endl;
    //Obstacles
    Eigen::MatrixXd obs(2,1);
    obs(0,0)=50;
    obs(1,0)=50;
    mock_obstacle_list.push_back(obs);//Matlab name: posObstacles //position qnd speed of mock obstacles
    obs(0,0)=10;
    obs(1,0)=10;
    mock_obstacle_list.push_back(obs);
    obs(0,0)=45;
    obs(1,0)=65;
    mock_obstacle_list.push_back(obs);
    obs(0,0)=80;
    obs(1,0)=70;
    mock_obstacle_list.push_back(obs);

    std::cout << " " << std::endl;
    std::cout << "mock_obstacle_list: "<< std::endl;
    std::cout << " " << std::endl;
    for(int i=0; i<(int)mock_obstacle_list.size(); ++i){
        std::cout << mock_obstacle_list[i] << std::endl;
        std::cout << " " << std::endl;
    }

    std::cout << "size detected_obstacle_list_qhat : " << detected_obstacle_list_qhat.size() << std::endl;//Matlab name:  qhat
    std::cout << "size detected_obstacles : " << detected_obstacles.size() << std::endl;//Matlab name: detectedObstacles
    std::cout << "size collisioned_obstacle : " << collisioned_obstacle.size() << std::endl;//Matlab name: collisionedObstacle//Incle a memory of the previous encountered obstacles if not in direction mode
    direction_boat_obstacle = 0;//Matlab name: directionObstacleDetected
    std::cout << "direction_boat_obstacle : " << direction_boat_obstacle << std::endl;
    radius_obstacle = 10;//Matlab name: rq  //security radius around the obstacle
    std::cout << "radius_obstacle : " << radius_obstacle << std::endl;
    collision_avoidance_point(2,1);//Matlab name: avoidCollisionPoint // Point to follow when an obstacle is detected
    collision_avoidance_point(0,0)=potential_field_dim(0,0)=-100;
    collision_avoidance_point(1,0)=potential_field_dim(0,2)=-100;
    std::cout << " " << std::endl;
    std::cout << "collision_avoidance_point: "<< std::endl;
    std::cout << " " << std::endl;
    std::cout << collision_avoidance_point<< std::endl;
    std::cout << " " << std::endl;
    is_obstacle_detected = 1;//Matlab name: is_obstacle_detected //1 if an obstacle is detected
    std::cout << "is_obstacle_detected : " << is_obstacle_detected << std::endl;


    //Targets
    Eigen::MatrixXd tar(2,1);
    tar(0,0)=100;
    tar(1,0)=100;
    target_list.push_back(tar);//Matlab name: posWaypoints
    tar(0,0)=0;
    tar(1,0)=0;
    target_list.push_back(tar);

    std::cout << " " << std::endl;
    std::cout << "target_list: "<< std::endl;
    std::cout << " " << std::endl;
    for(int i=0; i<(int)target_list.size(); ++i){
        std::cout <<target_list[i] << std::endl;
        std::cout << " " << std::endl;
    }

    target_phat = target_list[0];//Matlab name: phat
    std::cout << " " << std::endl;
    std::cout << "target_phat: "<< std::endl;
    std::cout << " " << std::endl;
    std::cout << target_phat<< std::endl;
    std::cout << " " << std::endl;


    //Boat
    boat_state(1,5);//Matlab name: x
    boat_state(0,0)=-10;//x
    boat_state(0,1)=-10;//y
    boat_state(0,2)=M_PI/4.0;//heading
    boat_state(0,3)=5;//speed
    boat_state(0,4)=0;//acceleration angle
    std::cout << " " << std::endl;
    std::cout << "boat_state: "<< std::endl;
    std::cout << " " << std::endl;
    std::cout << boat_state<< std::endl;
    std::cout << " " << std::endl;

    mock_detection_distance = 10 ;//Matlab name: distDetect//Range of detection of obstacles
    std::cout << "mock_detection_distance : " << mock_detection_distance << std::endl;
    mock_detection_angle = M_PI/8.0 ;//Matlab name: angleDetect//Angle of detection of obstacles
    std::cout << "mock_detection_angle : " << mock_detection_angle << std::endl;

    Eigen::MatrixXd li(2,1);//Matlab name: followedLine
    li(0,0)=boat_state(0,0);
    li(1,0)=boat_state(0,1);
    line_to_follow.push_back(li);
    li(0,0)=target_phat(0,0);
    li(1,0)=target_phat(1,0);
    line_to_follow.push_back(li);

    std::cout << " " << std::endl;
    std::cout << "line_to_follow: "<< std::endl;
    std::cout << " " << std::endl;
    for(int i=0; i<(int)line_to_follow.size(); ++i){
        std::cout <<line_to_follow[i] << std::endl;
        std::cout << " " << std::endl;
    }

    radius_corridor = 10;//Matlab name: r//corridor to stay in during line following
    std::cout << "radius_corridor : " << radius_corridor << std::endl;



    //World
    wind_direction = 3;//Matlab name: psi
    std::cout << "wind_direction : " << wind_direction << std::endl;

    //Different mode
    only_direction_mode = 0;//Matlab name:  onlyHeadingMode //Do the boat know the position or the direction of the obstacle?
    have_to_avoid_obstacle = 1;//Matlab name: haveToAvoidObstacle //In direction mode do the boat need to avoid an obstacle?
    can_compute_a_new_avoidance_point = 1;//Matlab name: avoidMode //Can the boat compute a new collision_avoidance_point
    std::cout << "only_direction_mode : " << only_direction_mode << std::endl;
    std::cout << "have_to_avoid_obstacle : " << have_to_avoid_obstacle << std::endl;
    std::cout << "can_compute_a_new_avoidance_point : " << can_compute_a_new_avoidance_point << std::endl;
    std::cout << " " << std::endl;
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
