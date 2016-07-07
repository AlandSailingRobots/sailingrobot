#include "CollisionAvoidanceBehave.h"
CollisionAvoidanceBehave::CollisionAvoidanceBehave(DBHandler *db):
  RoutingBehaviour(db)//super class call
{
    potential_field_dim = Eigen::MatrixXd::Zero(1,4);//Matlab name: dim //xmin xmax ymin ymax
    potential_field_dim(0,0)=-100;
    potential_field_dim(0,1)=100;
    potential_field_dim(0,2)=-100;
    potential_field_dim(0,3)=100;
    int step=5;
    int rows_Z = (int)(abs(potential_field_dim(0,2)-potential_field_dim(0,3))/step)+1;
    int cols_Z = (int)(abs(potential_field_dim(0,2)-potential_field_dim(0,3))/step)+1;
    potential_Z = Eigen::MatrixXd::Zero(rows_Z,cols_Z);//Matlab name: Z
    Eigen::VectorXd v = Eigen::VectorXd::LinSpaced(cols_Z,potential_field_dim(0,0),potential_field_dim(0,1));
    point_x = Eigen::MatrixXd::Zero(rows_Z,cols_Z);//Matlab name: P1
    point_y = Eigen::MatrixXd::Zero(rows_Z,cols_Z);//Matlab name: P2
    for(int i=0; i<rows_Z; i++){
        point_x.row(i)=v.adjoint();
        point_y.col(i)=v;
    }
    step_coeff=3;//Matlab name: eta //In direction mode length of the repulsion of the obstacle

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

    radius_obstacle = 10;//Matlab name: rq  //security radius around the obstacle
    collision_avoidance_point(2,1);//Matlab name: avoidCollisionPoint // Point to follow when an obstacle is detected
    collision_avoidance_point(0,0)=potential_field_dim(0,0);
    collision_avoidance_point(1,0)=potential_field_dim(0,2);
    is_obstacle_detected = 1;//Matlab name: is_obstacle_detected //1 if an obstacle is detected

    //Targets
    Eigen::MatrixXd tar(2,1);
    tar(0,0)=100;
    tar(1,0)=100;
    target_list.push_back(tar);//Matlab name: posWaypoints
    tar(0,0)=0;
    tar(1,0)=0;
    target_list.push_back(tar);
    target_phat = target_list[0];//Matlab name: phat

    //Boat
    boat_state(1,5);//Matlab name: x
    boat_state(0,0)=-10;//x
    boat_state(0,1)=-10;//y
    boat_state(0,2)=M_PI/4.0;//heading
    boat_state(0,3)=5;//speed
    boat_state(0,4)=0;//acceleration angle
    mock_detection_distance = 10 ;//Matlab name: distDetect//Range of detection of obstacles
    mock_detection_angle = M_PI/8.0 ;//Matlab name: angleDetect//Angle of detection of obstacles
    Eigen::MatrixXd li(2,1);
    li(0,0)=boat_state(0,0);
    li(1,0)=boat_state(0,1);
    line_to_follow.push_back(li);//Matlab name: followedLine
    li(0,0)=target_phat(0,0);
    li(1,0)=target_phat(1,0);
    line_to_follow.push_back(li);
    radius_corridor = 10;//Matlab name: r//corridor to stay in during line following

    //World
    wind_direction = 3;//Matlab name: psi

    //Different mode
    only_direction_mode = 0;//Matlab name:  onlyHeadingMode //Do the boat know the position or the direction of the obstacle?
    have_to_avoid_obstacle = 1;//Matlab name: haveToAvoidObstacle //In direction mode do the boat need to avoid an obstacle?
    can_compute_a_new_avoidance_point = 1;//Matlab name: avoidMode //Can the boat compute a new collision_avoidance_point

    printCollisionAvoidanceBehave(rows_Z,cols_Z,v);
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

//Matlab name: obstacle_detection
void CollisionAvoidanceBehave::mockObstacleDetection(){
    std::vector<Eigen::MatrixXd> a;
    std::vector<float> b;
    detected_obstacles=a;
    direction_boat_obstacle=b;
    float theta0 = 0;
    for(int i=0; i<(int)mock_obstacle_list.size();i++){
        if(sqrt(pow((mock_obstacle_list[i](0,0)-boat_state(0,0)),2)+pow((mock_obstacle_list[i](1,0)-boat_state(0,1)),2)) < mock_detection_distance ){
            theta0=atan2(mock_obstacle_list[i](1,0)-boat_state(0,1),mock_obstacle_list[i](0,0)-boat_state(0,0))-(fmod((boat_state(0,2)+M_PI),(2.0*M_PI))-M_PI);
            if(abs(theta0)<=(mock_detection_angle)){
                detected_obstacles.push_back(mock_obstacle_list[i]);
                direction_boat_obstacle.push_back(theta0);
            }
        }
    }
}
//Matlab name: calculate_potField
void CollisionAvoidanceBehave::calculatePotentialField(){
    //Init
    Eigen::MatrixXd ObsP = 0*point_x, ObjP = 0*point_x, xObs = 0*point_x, yObs = 0*point_x, xObj = 0*point_x, yObj = 0*point_x, xo = 0*point_x, yo = 0*point_x;
    Eigen::MatrixXd tHoleR = 0*point_x, tHoleL = 0*point_x, tPike = 0*point_x, tR = 0*point_x, tL = 0*point_x, tP = 0*point_x, ep1 = 0*point_x,ep2 = 0*point_x, xb = 0*point_x;
    Eigen::MatrixXd yb = 0*point_x, x1 = 0*point_x, y1 = 0*point_x, xw = 0*point_x, yw = 0*point_x, WindP = 0*point_x, BoatP = 0*point_x;
    float boatHeading =  boat_state(0,2), bearingObstacle = 0, T = 0,Ao = 1.6, strengthBoat = 3, strengthHoleBoat = 1.5, strengthPikeBoat = 5, lengthObstacle=radius_obstacle*step_coeff;
    int scaleHole = 50, scalePike = 550, strengthHoles = 2, strengthPike = 4, strength = 5, isObsPEmpty = 1;

    //Obstacle potential function
    if(only_direction_mode==0){
        if(detected_obstacle_list_qhat.size()!=0){
            isObsPEmpty = 0;
            for(int i=0;i<(int)detected_obstacle_list_qhat.size();i++){
                xObs = point_x.array()-detected_obstacle_list_qhat[i](0,0);
                yObs = point_y.array()-detected_obstacle_list_qhat[i](1,0);
                T = (boatHeading+M_PI/2)*0.3+(atan2(target_phat(0,0)-detected_obstacle_list_qhat[i](0,0),target_phat(1,0)-detected_obstacle_list_qhat[i](1,0))+M_PI/2)*0.7;
                xo =  xObs*cos(T) + yObs*sin(T);
                yo = -xObs*sin(T) + yObs*cos(T);
                tHoleR = ((xo.array()-35).array().square()+(yo*1).array().square())/scaleHole;
                tHoleL = ((xo.array()+35).array().square()+(yo*1).array().square())/scaleHole;
                tPike = ((xo*1).array().square()+(yo*1).array().square())/scalePike;

                tR=(-(tHoleR).array().square()).array().exp();
                tL=(-(tHoleL).array().square()).array().exp();
                tP=(-(tPike).array().square()).array().exp();
                ObsP = ObsP + strength*(strengthHoles*(-tR-tL)+strengthPike*tP);
            }
        }
    }
    else if(only_direction_mode==1){
        if( have_to_avoid_obstacle == 1){
             isObsPEmpty = 0;
             bearingObstacle=boat_state(0,2)+direction_boat_obstacle[0]-M_PI/2;

             x1 = point_x.array()-boat_state(0,0);
             y1 = point_y.array()-boat_state(0,1);
             xb =  x1*cos(bearingObstacle) + y1*sin(bearingObstacle);
             yb = -x1*sin(bearingObstacle) + y1*cos(bearingObstacle);
             ObsP = 15*rectangularPulse(-lengthObstacle,lengthObstacle,xb).array() * heavisideMat(yb).array()-5*rectangularPulse(-lengthObstacle*2,lengthObstacle*2,xb).array() *  heavisideMat(yb).array();
        }
    }

    //Objective potential function
    ObjP = Ao*((-(point_x.array()-target_phat(0,0)).array().square() - (point_y.array()-target_phat(1,0)).array().square())/25000).array().exp();
    // Wind preference potential function
    x1 = point_x.array()-boat_state(0,0);
    y1 = point_y.array()-boat_state(0,1);
    T = wind_direction+3.0*M_PI/4.0;
    xw =  x1*cos(T) + y1*sin(T);
    yw = -x1*sin(T) + y1*cos(T);
    WindP = 3*atanMat(xw).array() * heavisideMat(xw).array() * atanMat(yw).array() * heavisideMat(yw).array();

    // Boat preference
    ep1 = ((-(point_x.array()-boat_state(0,0)).array().square() - (point_y.array()-boat_state(0,1)).array().square())/4000).array().exp();
    ep2 = ((-(point_x.array()-boat_state(0,0)).array().square() - (point_y.array()-boat_state(0,1)).array().square())/200).array().exp();
    BoatP = strengthBoat*(-strengthHoleBoat*ep1 + strengthPikeBoat*ep2);

    //Computation of the field
    if(isObsPEmpty==1){
        potential_Z = 10*(-ObjP);
    }
    else{
        potential_Z = ObsP - ObjP + BoatP + WindP;
    }
    printMat("ObsP",ObsP);
    printMat("ObjP",ObjP);
    printMat("BoatP",BoatP);
    printMat("WindP",WindP);
    printMat("potential_Z",potential_Z);

}

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


void CollisionAvoidanceBehave::printStdVectorMat(std::string const& name, std::vector<Eigen::MatrixXd> const& v){

    std::cout << " " << std::endl;
    std::cout << name << " : " << std::endl;
    std::cout << " " << std::endl;
    for(int i=0; i<(int)v.size(); ++i){
        std::cout <<v[i] << std::endl;
        std::cout << " " << std::endl;
    }
}

void printStdVectorFloat(std::string const& name, std::vector<float> const& v){

    std::cout << " " << std::endl;
    std::cout << name << " : " << std::endl;
    std::cout << " " << std::endl;
    for(int i=0; i<(int)v.size(); ++i){
        std::cout <<v[i] << std::endl;
        std::cout << " " << std::endl;
    }
}

void CollisionAvoidanceBehave::printMat(std::string const& name,Eigen::MatrixXd const& mat){

    std::cout << " " << std::endl;
    std::cout << name << " : " << std::endl;
    std::cout << " " << std::endl;
    std::cout << mat<< std::endl;
    std::cout << " " << std::endl;
}

void  CollisionAvoidanceBehave::printCollisionAvoidanceBehave(int rows_Z,int cols_Z,Eigen::VectorXd v){
    printMat("potential_field_dim",potential_field_dim);
    std::cout << "rows_Z: "<< rows_Z << std::endl;
    std::cout << " " << std::endl;
    std::cout << "cols_Z: "<< cols_Z << std::endl;
    std::cout << " " << std::endl;
    printMat("potential_Z",potential_Z);
    std::cout << "v "<< v.adjoint() << std::endl;
    printMat("point_x",point_x);
    printMat("point_y",point_y);
    std::cout << "step_coeff : " << step_coeff << std::endl;
    std::cout << " " << std::endl;
    printStdVectorMat("mock_obstacle_list",mock_obstacle_list);
    std::cout << "size detected_obstacle_list_qhat : " << detected_obstacle_list_qhat.size() << std::endl;
    std::cout << "size detected_obstacles : " << detected_obstacles.size() << std::endl;
    std::cout << "size collisioned_obstacle : " << collisioned_obstacle.size() << std::endl;
    std::cout << "size direction_boat_obstacle : " << direction_boat_obstacle.size() << std::endl;
    std::cout << "radius_obstacle : " << radius_obstacle << std::endl;
    printMat("collision_avoidance_point",collision_avoidance_point);
    std::cout << "is_obstacle_detected : " << is_obstacle_detected << std::endl;
    printStdVectorMat("target_list",target_list);
    printMat("target_phat",target_phat);
    printMat("boat_state",boat_state);
    std::cout << "mock_detection_distance : " << mock_detection_distance << std::endl;
    std::cout << "mock_detection_angle : " << mock_detection_angle << std::endl;
    printStdVectorMat("line_to_follow",line_to_follow);
    std::cout << "radius_corridor : " << radius_corridor << std::endl;
    std::cout << "wind_direction : " << wind_direction << std::endl;
    std::cout << "only_direction_mode : " << only_direction_mode << std::endl;
    std::cout << "have_to_avoid_obstacle : " << have_to_avoid_obstacle << std::endl;
    std::cout << "can_compute_a_new_avoidance_point : " << can_compute_a_new_avoidance_point << std::endl;
    std::cout << " " << std::endl;
}
Eigen::MatrixXd CollisionAvoidanceBehave::atanMat(Eigen::MatrixXd mat){
    for(int i = 0; i<mat.rows();i++){
        for(int j = 0; j<mat.cols();j++){
            mat(i,j)=atan(mat(i,j));
        }
    }
    return mat;
}
float CollisionAvoidanceBehave::heaviside(float num){
    if(num < 0){
        num = 0;
    }
    else if(num > 0){
        num = 1;
    }
    else{
        num= 0.5;
    }
    return num;
}
Eigen::MatrixXd CollisionAvoidanceBehave::heavisideMat(Eigen::MatrixXd mat){
    for(int i = 0; i<mat.rows();i++){
        for(int j = 0; j<mat.cols();j++){
            mat(i,j)=heaviside(mat(i,j));
        }
    }
    return mat;
}
Eigen::MatrixXd CollisionAvoidanceBehave::rectangularPulse(float a, float b, Eigen::MatrixXd x){
    for(int i = 0; i<x.rows();i++){
        for(int j = 0; j<x.cols();j++){
            if(a < x(i,j) && b > x(i,j)){
                x(i,j)=1;
            }
            if(x(i,j) < a || x(i,j) > b || a==b){
                x(i,j)=0;
            }
            if(a < b && (x(i,j)==a || x(i,j) ==b)){
                x(i,j)=0.5;
            }
        }
    }
    return x;
}
