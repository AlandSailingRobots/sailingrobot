#include <iostream>
#include <Eigen/Dense>
#include <math.h>
#include <cmath>
#include <stdint.h> //uint8_t
#include <vector>
#include <iostream>

Eigen::MatrixXd atanMat(Eigen::MatrixXd mat){
    for(int i = 0; i<mat.rows();i++){
        for(int j = 0; j<mat.cols();j++){
            mat(i,j)=atan(mat(i,j));
        }
    }
    return mat;
}
float heaviside(float num){
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
    Eigen::MatrixXd heavisideMat(Eigen::MatrixXd mat){
    for(int i = 0; i<mat.rows();i++){
        for(int j = 0; j<mat.cols();j++){
            mat(i,j)=heaviside(mat(i,j));
        }
    }
    return mat;
}

Eigen::MatrixXd rectangularPulse(float a, float b, Eigen::MatrixXd x){
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

void printStdVectorMat(std::string const& name, std::vector<Eigen::MatrixXd> const& v){

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

void printMat(std::string const& name,Eigen::MatrixXd const& mat){

    std::cout << " " << std::endl;
    std::cout << name << " : " << std::endl;
    std::cout << " " << std::endl;
    std::cout << mat<< std::endl;
    std::cout << " " << std::endl;
}

void  printCollisionAvoidanceBehave(Eigen::MatrixXd potential_Z,Eigen::MatrixXd point_x,Eigen::MatrixXd point_y,float step_coeff,Eigen::MatrixXd potential_field_dim,std::vector<Eigen::MatrixXd> mock_obstacle_list,
    std::vector<Eigen::MatrixXd> detected_obstacle_list_qhat,std::vector<Eigen::MatrixXd> detected_obstacles, std::vector<Eigen::MatrixXd> collisioned_obstacle, std::vector<float> direction_boat_obstacle,float radius_obstacle,   Eigen::MatrixXd collision_avoidance_point,int is_obstacle_detected,std::vector<Eigen::MatrixXd> target_list,Eigen::MatrixXd target_phat,Eigen::MatrixXd boat_state, float mock_detection_distance,float mock_detection_angle,
    std::vector<Eigen::MatrixXd> line_to_follow,float radius_corridor,float wind_direction,int only_direction_mode,int have_to_avoid_obstacle,int can_compute_a_new_avoidance_point,int rows_Z,int cols_Z,Eigen::VectorXd v){
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
void mockObstacleDetection(std::vector<Eigen::MatrixXd> const& mock_obstacle_list, std::vector<Eigen::MatrixXd>& detected_obstacles, std::vector<float>& direction_boat_obstacle,
                            Eigen::MatrixXd const& boat_state,float const& mock_detection_distance,float const& mock_detection_angle){
    std::cout <<""<<std::endl;
    std::cout << "$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$" << std::endl;
    std::cout << "ENTERING mockObstacleDetection" << std::endl;
    std::vector<Eigen::MatrixXd> a;
    std::vector<float> b;
    detected_obstacles=a;
    direction_boat_obstacle=b;
    float theta0 = 0;
    printStdVectorMat("detected_obstacles init",detected_obstacles);
    printStdVectorFloat("direction_boat_obstacle init",direction_boat_obstacle);
    for(int i=0; i<(int)mock_obstacle_list.size();i++){
        if(sqrt(pow((mock_obstacle_list[i](0,0)-boat_state(0,0)),2)+pow((mock_obstacle_list[i](1,0)-boat_state(0,1)),2)) < mock_detection_distance ){
            theta0=atan2(mock_obstacle_list[i](1,0)-boat_state(0,1),mock_obstacle_list[i](0,0)-boat_state(0,0))-(fmod((boat_state(0,2)+M_PI),(2.0*M_PI))-M_PI);
            std::cout << "DISTANCE BOAT OBSTACLE < MOCK DETECTION DISTANCE" << std::endl;
            std::cout << "theta0 relative angle boat obstacle : " << theta0 << std::endl;
            if(abs(theta0)<=(mock_detection_angle)){
                std::cout << "ANGLE BOAT OBSTACLE < MOCK DETECTION ANGLE" << std::endl;
                detected_obstacles.push_back(mock_obstacle_list[i]);
                direction_boat_obstacle.push_back(theta0);
                printStdVectorMat("detected_obstacles",detected_obstacles);
                printStdVectorFloat("direction_boat_obstacle",direction_boat_obstacle);
            }
        }
    }
    std::cout << "EXIT mockObstacleDetection" << std::endl;
}

//Matlab name: calculate_potField
void calculatePotentialField(float const& radius_obstacle, Eigen::MatrixXd const& point_x,Eigen::MatrixXd const& point_y, Eigen::MatrixXd const& boat_state,Eigen::MatrixXd const& target_phat,std::vector<Eigen::MatrixXd> const& detected_obstacle_list_qhat,float const& wind_direction,int const& only_direction_mode,int const& have_to_avoid_obstacle,float const& step_coeff,std::vector<float> direction_boat_obstacle, Eigen::MatrixXd& potential_Z){
    std::cout <<""<<std::endl;
    std::cout << "$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$" << std::endl;
    std::cout << "ENTERING calculatePotentialField" << std::endl;
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
    //printMat("ObsP",ObsP);
    //printMat("ObjP",ObjP);
    //printMat("BoatP",BoatP);
    //printMat("WindP",WindP);
    //printMat("potential_Z",potential_Z);
    std::cout << "EXIT calculatePotentialField" << std::endl;
}


void calculate_collision_avoidance_point(Eigen::MatrixXd potential_Z,Eigen::MatrixXd potential_field_dim,Eigen::MatrixXd& collision_avoidance_point){
        std::cout <<""<<std::endl;
        std::cout << "$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$" << std::endl;
        std::cout << "ENTERING calculate_collision_avoidance_point" << std::endl;
        int xmin = potential_field_dim(0,0);
        int xmax = potential_field_dim(0,1);
        int ymin = potential_field_dim(0,2);
        int ymax = potential_field_dim(0,3);
        float I_row = 0, I_col = 0;
        float minZ = potential_Z.minCoeff(&I_row, &I_col);
        float I_x = I_col*(ymax-ymin)/potential_Z.cols()-(abs(ymin));
        float I_y = I_row*(xmax-xmin)/potential_Z.rows()-(abs(xmin));
        collision_avoidance_point(0,0) = I_x ;
        collision_avoidance_point(1,0) = I_y ;
        std::cout << "minZ: " << minZ << std::endl;
        std::cout << "I_row: " << I_row << std::endl;
        std::cout << "I_col: " << I_col << std::endl;
        printMat("collision_avoidance_point", collision_avoidance_point);
        std::cout << "EXIT calculate_collision_avoidance_point" << std::endl;

}


void update_obstacles(std::vector<Eigen::MatrixXd> detected_obstacles,Eigen::MatrixXd boat_state,float mock_detection_distance,float mock_detection_angle,std::vector<Eigen::MatrixXd>& detected_obstacle_list_qhat){
    std::cout <<""<<std::endl;
    std::cout << "$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$" << std::endl;
    std::cout << "ENTERING update_obstacles" << std::endl;
    int i=0;
    float T = 0;
    printStdVectorMat("detected_obstacle_list_qhat",detected_obstacle_list_qhat);
    while(i<(int)detected_obstacle_list_qhat.size()){
        T = atan2((detected_obstacle_list_qhat[i](1,0)-boat_state(0,1)),(detected_obstacle_list_qhat[i](0,0)-boat_state(0,0)))-boat_state(0,2);
        std::cout << "T angle boat obstacle : " << T << std::endl;
        bool find1 = true, find2= true, qhatBelongs2DetObs = true;
        if(detected_obstacles.size()!=0){
            std::cout << "detected_obstacles number:" << i << std::endl;
            find1 = (detected_obstacles[0](0,0)==detected_obstacle_list_qhat[i](0,0));
            find2 = (detected_obstacles[0](1,0)==detected_obstacle_list_qhat[i](1,0));
            qhatBelongs2DetObs = (find1==find2);
            std::cout << "detected_obstacles belongs to the memory qhat:" << qhatBelongs2DetObs << std::endl;
        }

        else{
            qhatBelongs2DetObs = true ;
        }

        float norms = sqrt(pow((detected_obstacle_list_qhat[i](0,0)-boat_state(0,0)),2)+pow((detected_obstacle_list_qhat[i](1,0)-boat_state(0,1)),2));
        std::cout << "norm obstacle boat number :" << i << std::endl;

        if( ((norms <=mock_detection_distance) && (abs(T)<=mock_detection_angle)) && (qhatBelongs2DetObs) ){
            detected_obstacle_list_qhat.erase(detected_obstacle_list_qhat.begin() + i);
        }
        i=i+1;
    }

    int ctr = 0;
    for (int i = 0; i<(int)detected_obstacles.size(); i++){
        ctr = 0;
        for (int j=0;j<(int)detected_obstacle_list_qhat.size(); j++){
            if(detected_obstacles[i]==detected_obstacle_list_qhat[j]){
                ctr = ctr + 1;
            }
        }
        if(ctr==0){
            detected_obstacle_list_qhat.push_back(detected_obstacles[i]);
        }
    }
    printStdVectorMat("detected_obstacle_list_qhat",detected_obstacle_list_qhat);
    std::cout << "EXIT update_obstacles" << std::endl;
}//MATLAB name : update_obstacles

void obstacleOnACollisionCourse(Eigen::MatrixXd boat_state,std::vector<Eigen::MatrixXd> detected_obstacle_list_qhat,float radius_obstacle,float radius_corridor,int& can_compute_a_new_avoidance_point,
                                std::vector<Eigen::MatrixXd> detected_obstacles,std::vector<Eigen::MatrixXd>& collisioned_obstacle,int only_direction_mode,int& is_obstacle_detected,int& have_to_avoid_obstacle){
    std::vector<Eigen::MatrixXd> b;
    std::cout <<""<<std::endl;
    std::cout << "$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$" << std::endl;
    std::cout << "ENTERING obstacleOnACollisionCourse" << std::endl;
    collisioned_obstacle = b;
    float distBoatObstacle = 0, coneAngle = 0,boatHeading = 0, obstacleHeading = 0;
    printStdVectorMat("detected_obstacle_list_qhat",detected_obstacle_list_qhat);
    for (int i=0; i<(int)detected_obstacle_list_qhat.size();i++){
        distBoatObstacle = sqrt(pow((detected_obstacle_list_qhat[i](0,0)-boat_state(0,0)),2)+pow((detected_obstacle_list_qhat[i](1,0)-boat_state(0,1)),2));
        std::cout << "DISTANCE BOAT OBSTACLE distBoatObstacle : " << distBoatObstacle << std::endl;
        coneAngle = atan2(radius_obstacle,distBoatObstacle);
        std::cout << "ANGLE RADIUS BOAT OBSTACLE coneAngle : " << coneAngle << std::endl;
        boatHeading = fmod(boat_state(0,2),2.0*M_PI);
        obstacleHeading = fmod(atan2(detected_obstacle_list_qhat[i](0,0)-boat_state(0,0),detected_obstacle_list_qhat[i](1,0)-boat_state(0,1)),2.0*M_PI);
        std::cout << "obstacleHeading  : " << obstacleHeading << std::endl;
        std::cout << "boatHeading  : " << boatHeading << std::endl;

        if((abs(boatHeading-obstacleHeading) < coneAngle) && (distBoatObstacle < (radius_corridor*2)) && (boat_state(0,3)>1)){
            if(collisioned_obstacle.size()==0){
                Eigen::MatrixXd a(2,1);
                collisioned_obstacle.push_back(a);
            }
            collisioned_obstacle[0] = detected_obstacle_list_qhat[i];
            printStdVectorMat("collisioned_obstacle",collisioned_obstacle);
        }
    }

    if(only_direction_mode==1){
        collisioned_obstacle.push_back(detected_obstacles[0]);
    }
     if(detected_obstacles.size()!=0){
        have_to_avoid_obstacle=1;
    }


    else{
        have_to_avoid_obstacle=0;
    }
    if(detected_obstacles.size()==0){
        is_obstacle_detected=0;
    }
    else if(detected_obstacles.size()!=0 && is_obstacle_detected==0){
        is_obstacle_detected=1;
        can_compute_a_new_avoidance_point=1;
    }
    std::cout << "EXIT obstacleOnACollisionCourse" << std::endl;
}//Matlab name: boat_on_collision_course


void printSimul(Eigen::MatrixXd boat_state,Eigen::MatrixXd collision_avoidance_point,std::vector<Eigen::MatrixXd> collisioned_obstacle,Eigen::MatrixXd target_phat){

    int xb =(int)(boat_state(0,0));
    int yb =(int)(boat_state(0,1));
    int xt =(int)(target_phat(0,0));
    int yt =(int)(target_phat(1,0));

    int xI =(int)(collision_avoidance_point(0,0));
    int yI =(int)(collision_avoidance_point(1,0));
    int xo=0, yo=0;
    if(collisioned_obstacle.size()!=0){
        xo = (int)(collisioned_obstacle[0](0,0));
        yo = (int)(collisioned_obstacle[0](1,0));
    }
    std::cout <<""<<std::endl;
    std::cout << "$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$" << std::endl;
    std::cout << "$$$$ BOAT POSITION $$$$" <<"$$$$ OBSTACLE POSITION $$$$" << "$$$$ WAYPOINT  POSITION $$$$" <<"$$$$ TARGET  POSITION $$$$" <<std::endl;
    std::cout << "$$$$   ("<< xb <<","<<yb<<")"<<"   $$$$" <<"$$$$     ("<< xo <<","<<yo<<")"<<"       $$$$" << "$$$$      ("<< xI <<","<<yI<<")"<<"      $$$$" << "$$$$     ("<< xt <<","<<yt<<")"<<"    $$$$" <<std::endl;
    std::cout << "$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$" << std::endl;


}

void avoidObstacle(Eigen::MatrixXd target_phat,Eigen::MatrixXd boat_state,Eigen::MatrixXd point_x,Eigen::MatrixXd point_y,float wind_direction,float radius_obstacle,float radius_corridor,
                    int only_direction_mode,int have_to_avoid_obstacle,float step_coeff,std::vector<float> direction_boat_obstacle,int& can_compute_a_new_avoidance_point,std::vector<Eigen::MatrixXd>& line_to_follow,std::vector<Eigen::MatrixXd> collisioned_obstacle,
                    std::vector<Eigen::MatrixXd>& detected_obstacle_list_qhat,Eigen::MatrixXd potential_Z,Eigen::MatrixXd potential_field_dim,Eigen::MatrixXd& collision_avoidance_point){
    std::cout <<""<<std::endl;
    std::cout << "$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$" << std::endl;
    std::cout << "ENTERING avoidObstacle" << std::endl;
    float norma=0;
    float collLineAngle = 0;
    printStdVectorMat("collisioned_obstacle",collisioned_obstacle);
    if(detected_obstacle_list_qhat.size()!=0 && collisioned_obstacle.size()!=0 && can_compute_a_new_avoidance_point==1)

        calculatePotentialField(radius_obstacle,point_x, point_y,  boat_state, target_phat, detected_obstacle_list_qhat, wind_direction, only_direction_mode, have_to_avoid_obstacle, step_coeff, direction_boat_obstacle, potential_Z);
        calculate_collision_avoidance_point(potential_Z, potential_field_dim, collision_avoidance_point);
        printStdVectorMat("collisioned_obstacle",collisioned_obstacle);

       norma= (sqrt(pow((collisioned_obstacle[0](0,0)-boat_state(0,0)),2)+pow((collisioned_obstacle[0](1,0)-boat_state(0,1)),2))+radius_corridor+radius_obstacle);

        std::cout <<"norma : " << norma <<std::endl;
        Eigen::MatrixXd startCollLine(2,1);
        startCollLine(0,0)=norma*cos(boat_state(0,2)+M_PI)+boat_state(0,0);
        startCollLine(1,0)=norma*sin(boat_state(0,2)+M_PI)+boat_state(0,1);


        line_to_follow[0]= startCollLine;
        line_to_follow[1]= collision_avoidance_point;
        can_compute_a_new_avoidance_point = 0;
        printMat("startCollLine",startCollLine);
        printStdVectorMat("line_to_follow",line_to_follow);

    collLineAngle = atan2(startCollLine(1,0)-collision_avoidance_point(1,0),startCollLine(0,0)-collision_avoidance_point(0,0));
    if(     (sqrt(pow((boat_state(0,0)-collision_avoidance_point(0,0)),2)+pow((boat_state(0,1)-collision_avoidance_point(1,0)),2))<(2*radius_corridor/10)
            || ( ((boat_state(0,0)-collision_avoidance_point(0,0))*cos(collLineAngle)+(boat_state(0,1)-collision_avoidance_point(1,0))*sin(collLineAngle)>-40)
                && ((boat_state(0,0)-collision_avoidance_point(0,0))*cos(collLineAngle)+(boat_state(0,1)-collision_avoidance_point(1,0))*sin(collLineAngle)<0)
                && (abs(-(boat_state(0,0)-collision_avoidance_point(0,0))*sin(collLineAngle)+(boat_state(0,1)-collision_avoidance_point(1,0))*cos(collLineAngle))<radius_corridor/2)) )
        && can_compute_a_new_avoidance_point == 0){

          line_to_follow[0] = collision_avoidance_point;
          line_to_follow[1] = target_phat;
          can_compute_a_new_avoidance_point = 1;
          printStdVectorMat("line_to_follow",line_to_follow);
      }

    if(only_direction_mode==1){
        std::vector<Eigen::MatrixXd> b;
        detected_obstacle_list_qhat=b;
        printStdVectorMat("detected_obstacle_list_qhat",detected_obstacle_list_qhat);
    }
    std::cout << "EXIT avoidObstacle" << std::endl;
}//Matlab name: avoid_obstacle
int sign(float num){
    int signNum=0;
    if (num >0){
        signNum=1;
    }
    else if (num<0){
        signNum=-1;
    }
    else{
        signNum=0;
    }
    return signNum;
}
Eigen::MatrixXd f(Eigen::MatrixXd boat_state,Eigen::MatrixXd inputs,int wind_speed,float wind_direction){
    float p1=0.1, p2=1, p3=6000, p4=1000, p5=2000;
    float p6=1, p7=1, p8=2, p9=300, p10=6000;
    float theta=boat_state(0,2), v=boat_state(0,3), w=boat_state(0,4), deltar=inputs(0,0), deltasmax=inputs(1,0);
    Eigen::MatrixXd w_ap(2,1);
    w_ap(0,0)=wind_speed*cos(wind_direction-theta)-v;
    w_ap(1,0)=wind_speed*sin(wind_direction-theta);
    float psi_ap=atan2(w_ap(0,0),w_ap(1,0));
    float a_ap=sqrt(pow(w_ap(0,0),2)+pow(w_ap(1,0),2));
    float sigma=cos(psi_ap)+cos(deltasmax);
    float deltas=0;
    if (sigma<0){ deltas=M_PI+psi_ap;}
    else if(sigma>=0){ deltas=-sign(sin(psi_ap))*deltasmax;}

    float fr = p5*v*sin(deltar),  fs = p4*a_ap*sin(deltas-psi_ap);
    float dx=v*cos(theta)+p1*wind_speed*cos(wind_speed);
    float dy=v*sin(theta)+p1*wind_speed*sin(wind_speed);
    float dtheta=w;
    float dv=(1/p9)*(sin(deltas)*fs-sin(deltar)*fr-p2*pow(v,2));
    float dw=(1/p10)*((p6-p7*cos(deltas))*fs-p8*cos(deltar)*fr-p3*w*v);
    Eigen::MatrixXd xdot(1,5);//Matlab name: x
    boat_state(0,0)=dx;//x
    boat_state(0,1)=dy;//y
    boat_state(0,2)=dtheta;//heading
    boat_state(0,3)=dv;//speed
    boat_state(0,4)=dw;//acceleration angle
    return xdot;
}

int main()
{
    Eigen::MatrixXd potential_field_dim = Eigen::MatrixXd::Zero(1,4);//Matlab name: dim //xmin xmax ymin ymax
    potential_field_dim(0,0)=-100;
    potential_field_dim(0,1)=100;
    potential_field_dim(0,2)=-100;
    potential_field_dim(0,3)=100;

    int step=5;
    int rows_Z = (int)(abs(potential_field_dim(0,2)-potential_field_dim(0,3))/step)+1;
    int cols_Z = (int)(abs(potential_field_dim(0,2)-potential_field_dim(0,3))/step)+1;
    Eigen::MatrixXd potential_Z = Eigen::MatrixXd::Zero(rows_Z,cols_Z);//Matlab name: Z
    Eigen::VectorXd v = Eigen::VectorXd::LinSpaced(cols_Z,potential_field_dim(0,0),potential_field_dim(0,1));

    Eigen::MatrixXd point_x = Eigen::MatrixXd::Zero(rows_Z,cols_Z);//Matlab name: P1
    Eigen::MatrixXd point_y = Eigen::MatrixXd::Zero(rows_Z,cols_Z);//Matlab name: P2

    for(int i=0; i<rows_Z; i++){
        point_x.row(i)=v.adjoint();
        point_y.col(i)=v;
    }

    float step_coeff=3;//Matlab name: eta //In direction mode length of the repulsion of the obstacle

    //Obstacles
    std::vector<Eigen::MatrixXd> mock_obstacle_list;//Matlab name: posObstacles //position qnd speed of mock obstacles
    Eigen::MatrixXd obs(2,1);
    obs(0,0)=-5;
    obs(1,0)=-5;
    mock_obstacle_list.push_back(obs);
    obs(0,0)=10;
    obs(1,0)=10;
    mock_obstacle_list.push_back(obs);
    obs(0,0)=45;
    obs(1,0)=65;
    mock_obstacle_list.push_back(obs);
    obs(0,0)=80;
    obs(1,0)=70;
    mock_obstacle_list.push_back(obs);

    std::vector<Eigen::MatrixXd> detected_obstacle_list_qhat;//Matlab name:  qhat

    std::vector<Eigen::MatrixXd> detected_obstacles;//Matlab name: detectedObstacles

    std::vector<Eigen::MatrixXd> collisioned_obstacle;//Matlab name: collisionedObstacle//Incle a memory of the previous encountered obstacles if not in direction mode

    std::vector<float> direction_boat_obstacle;//Matlab name: directionObstacleDetected

    float radius_obstacle = 10;//Matlab name: rq  //security radius around the obstacle

    Eigen::MatrixXd collision_avoidance_point(2,1);//Matlab name: avoidCollisionPoint // Point to follow when an obstacle is detected
    collision_avoidance_point(0,0)=potential_field_dim(0,0);
    collision_avoidance_point(1,0)=potential_field_dim(0,2);

    int is_obstacle_detected = 1;//Matlab name: is_obstacle_detected //1 if an obstacle is detected

    //Targets
    std::vector<Eigen::MatrixXd> target_list;//Matlab name: posWaypoints
    Eigen::MatrixXd tar(2,1);
    tar(0,0)=100;
    tar(1,0)=100;
    target_list.push_back(tar);
    tar(0,0)=0;
    tar(1,0)=0;
    target_list.push_back(tar);

    Eigen::MatrixXd target_phat = target_list[0];//Matlab name: phat

    //Boat
    Eigen::MatrixXd boat_state(1,5);//Matlab name: x
    boat_state(0,0)=-10;//x
    boat_state(0,1)=-10;//y
    boat_state(0,2)=M_PI/4;//heading
    boat_state(0,3)=5;//speed
    boat_state(0,4)=0;//acceleration angle

    Eigen::MatrixXd inputs(2,1);
    inputs(0,0)=-0.1;
    inputs(1,0)=0.02;

    float mock_detection_distance = 10 ;//Matlab name: distDetect//Range of detection of obstacles

    float mock_detection_angle = M_PI/8.0 ;//Matlab name: angleDetect//Angle of detection of obstacles

    std::vector<Eigen::MatrixXd> line_to_follow;//Matlab name: followedLine
    Eigen::MatrixXd li(2,1);
    li(0,0)=boat_state(0,0);
    li(1,0)=boat_state(0,1);
    line_to_follow.push_back(li);
    li(0,0)=target_phat(0,0);
    li(1,0)=target_phat(1,0);
    line_to_follow.push_back(li);

    float radius_corridor = 10;//Matlab name: r//corridor to stay in during line following

    //World
    int wind_speed = 3;
    float wind_direction = 1;//Matlab name: psi

    //Different mode
    int only_direction_mode = 0;//Matlab name:  onlyHeadingMode //Do the boat know the position or the direction of the obstacle?
    int have_to_avoid_obstacle = 1;//Matlab name: haveToAvoidObstacle //In direction mode do the boat need to avoid an obstacle?
    int can_compute_a_new_avoidance_point = 1;//Matlab name: avoidMode //Can the boat compute a new collision_avoidance_point

    printCollisionAvoidanceBehave(potential_Z,point_x,point_y,step_coeff,potential_field_dim, mock_obstacle_list,detected_obstacle_list_qhat,detected_obstacles,
                                  collisioned_obstacle,direction_boat_obstacle,radius_obstacle, collision_avoidance_point,is_obstacle_detected,target_list, target_phat, boat_state,
                                  mock_detection_distance,mock_detection_angle,line_to_follow,radius_corridor,wind_direction,only_direction_mode,have_to_avoid_obstacle,can_compute_a_new_avoidance_point,rows_Z,cols_Z,v);

    //Function tests
    //mockObstacleDetection(mock_obstacle_list, detected_obstacles, direction_boat_obstacle,boat_state, mock_detection_distance, mock_detection_angle);
    //printStdVectorMat("mock_obstacle_list",mock_obstacle_list);
    //printStdVectorMat("detected_obstacles",detected_obstacles);
    //printStdVectorFloat("direction_boat_obstacle",direction_boat_obstacle);

    Eigen::MatrixXd test_atan_x = Eigen::MatrixXd::Ones(rows_Z,cols_Z);
    printMat("test_atan_x",test_atan_x);
    Eigen::MatrixXd t = atanMat(test_atan_x);
    printMat("test_atan_x",t);

    Eigen::MatrixXd test_heav_x = Eigen::MatrixXd::Random(rows_Z,cols_Z);
    printMat("test_heav_x",test_heav_x);
    Eigen::MatrixXd he = heavisideMat(test_heav_x);
    printMat("test_heav_x",he);

    Eigen::MatrixXd x1 = point_x.array() - boat_state(0,0);
    Eigen::MatrixXd y1 = point_y.array()-boat_state(0,1);
    Eigen::MatrixXd xb =  x1*cos(M_PI/4.0) + y1*sin(M_PI/4.0);
    Eigen::MatrixXd yb = -x1*sin(M_PI/4.0) + y1*cos(M_PI/4.0);
    printMat("test_rectP_x",xb);
    Eigen::MatrixXd ObsP = rectangularPulse(-10*2,10*2,xb);
    printMat("test_rectP_x",ObsP);
    float tim=0.0;
    float dt=0.01;
    while(tim<100){

        mockObstacleDetection(mock_obstacle_list, detected_obstacles, direction_boat_obstacle,boat_state, mock_detection_distance, mock_detection_angle);
        //calculatePotentialField( radius_obstacle,point_x, point_y,  boat_state, target_phat, detected_obstacle_list_qhat, wind_direction, only_direction_mode, have_to_avoid_obstacle, step_coeff, direction_boat_obstacle, potential_Z);
        update_obstacles( detected_obstacles, boat_state, mock_detection_distance, mock_detection_angle, detected_obstacle_list_qhat);
        obstacleOnACollisionCourse(boat_state,detected_obstacle_list_qhat, radius_obstacle, radius_corridor, can_compute_a_new_avoidance_point,detected_obstacles, collisioned_obstacle, only_direction_mode, is_obstacle_detected, have_to_avoid_obstacle);
        //calculate_collision_avoidance_point(potential_Z,potential_field_dim,collision_avoidance_point);
        avoidObstacle(target_phat, boat_state, point_x, point_y, wind_direction,radius_obstacle, radius_corridor, only_direction_mode,have_to_avoid_obstacle, step_coeff, direction_boat_obstacle, can_compute_a_new_avoidance_point, line_to_follow,collisioned_obstacle,
                        detected_obstacle_list_qhat, potential_Z,potential_field_dim,collision_avoidance_point);
        printSimul( boat_state, collision_avoidance_point,collisioned_obstacle,target_phat);
        boat_state= boat_state + f(boat_state, inputs, wind_speed,wind_direction)*dt;
        tim=tim+dt;
    }
}
