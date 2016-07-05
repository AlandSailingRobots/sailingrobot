#include "CollisionAvoidanceBehave.h"
CollisionAvoidanceBehave::CollisionAvoidanceBehave(DBHandler *db):
  RoutingBehaviour(db)//super class call
{
}

bool CollisionAvoidanceBehave::init()
{

    return true;
}

void CollisionAvoidanceBehave::computeCommands(SystemStateModel &systemStateModel,std::unique_ptr<Position> const& position,
                                  double trueWindDirection, bool mockPosition,
                                  bool getHeadingFromCompass){
}


void CollisionAvoidanceBehave::manageDatabase(double trueWindDirection,SystemStateModel &m_systemStateModel){

}
void CollisionAvoidanceBehave::setupWaypoints(){}
void CollisionAvoidanceBehave::setNextWaypoint(){}
void CollisionAvoidanceBehave::setPreviousWayPoint(SystemStateModel &m_systemStateModel){}

int CollisionAvoidanceBehave::getHeading(SystemStateModel &m_systemStateModel,bool m_mockPosition,bool m_getHeadingFromCompass,std::unique_ptr<Position> const& position) {
    return 1;
}
/*
MatrixXd CollisionAvoidanceBehave::createWall(MatrixXd const& starting_point,MatrixXd const& ending_point,float step){
    Eigen::MatrixXd m(2,1);
    return m;
}
void CollisionAvoidanceBehave::obstacleOnACollisionCourse(MatrixXd const& boat_state, MatrixXd const& detected_obstacle_list_qhat){}
void CollisionAvoidanceBehave::mockObstacleDetection(MatrixXd const& boat_state, MatrixXd const& mock_obstacle_list,std::vector<Eigen::MatrixXd> detected_obstacle_list_qhat){}

void CollisionAvoidanceBehave::calculatePotentialField(MatrixXd const& point_x,MatrixXd const& point_y,
                            MatrixXd const& boat_state,MatrixXd const& target_phat,
                            MatrixXd const& detected_obstacle_list_qhat,
                            MatrixXd& potential_Z){}
void CollisionAvoidanceBehave::calculateAvoidancePoint(MatrixXd const& boat_state, MatrixXd const& target_phat, MatrixXd const& detected_obstacle_list_qhat,MatrixXd const& potential_Z, MatrixXd& avoidance_point){}
*/
