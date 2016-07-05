#ifndef __COLLISIONAVOIDANCEBEHAVE_H__
#define __COLLISIONAVOIDANCEBEHAVE_H__
#include <stdint.h> //uint8_t
#include <math.h>
#include <algorithm>
#include <cmath>
#include "coursecalculation/CourseCalculation.h"
#include "RoutingBehaviour.h"
#include "utility/Utility.h"
#include "waypointrouting/WaypointRouting.h"
#include "behaviourclass/LineFollowBehaviour.h"
#include <libs/Eigen>
using Eigen::MatrixXd;

class CollisionAvoidanceBehave:public RoutingBehaviour{
public:
    CollisionAvoidanceBehave(DBHandler *db);
    ~CollisionAvoidanceBehave(){};

    bool init();

    void computeCommands(SystemStateModel &systemStateModel,std::unique_ptr<Position> const& position,
                                      double trueWindDirection, bool mockPosition,
                                      bool getHeadingFromCompass);  //make the computation of the commands return true if the computation was successfull

    void manageDatabase(double trueWindDirection, SystemStateModel &systemStateModel);
    double getRudderCommand();
    double getSailCommand();

private:

    void setupWaypoints();
    void setNextWaypoint();
    void setPreviousWayPoint(SystemStateModel &m_systemStateModel);
    int getHeading(SystemStateModel &m_systemStateModel,bool m_mockPosition,bool m_getHeadingFromCompass,std::unique_ptr<Position> const& position);
    //Potential Field
    MatrixXd potential_Z;
    MatrixXd point_x;
    MatrixXd point_y;
    float step_coeff;//In direction mode length of the repulsion of the obstacle
    MatrixXd potential_field_dim; //xmin xmax ymin ymax

    //Obstacles
    std::vector<MatrixXd> mock_obstacle_list;
    std::vector<MatrixXd> detected_obstacle_list_qhat;
    std::vector<MatrixXd> detected_obstacles;
    std::vector<MatrixXd> collisioned_obstacle;//Incle a memory of the previous encountered obstacles if not in direction mode
    float direction_boat_obstacle;
    float radius_obstacle;//security radius around the obstacle
    MatrixXd collision_avoidance_point;// Point to follow when an obstacle is detected

    //Targets
    std::vector<MatrixXd> target_list;
    MatrixXd target_phat;

    //Boat
    MatrixXd boat_state;
    float mock_detection_distance;
    std::vector<MatrixXd> line_to_follow;
    float radius_corridor;//corridor to stay in during line following

    //World
    float wind_direction;

    //Different mode
    int only_direction_mode; //Do the boat know the position or the direction of the obstacle?
    int have_to_avoid_obstacle;//In direction mode do the boat need to avoid an obstacle?
    int can_compute_a_new_avoidance_point;//Can the boat compute a new collision_avoidance_point
    /*
    void mockObstacleDetection(MatrixXd const& boat_state, std::vector<Eigen::MatrixXd> const& mock_obstacle_list,float mock_detection_distance,float mock_detection_distance,
                                std::vector<Eigen::MatrixXd>& detected_obstacles, float& direction_boat_obstacle);

    void calculatePotentialField(MatrixXd const& point_x,
                                 MatrixXd const& point_y,
                                 MatrixXd const& boat_state,
                                 MatrixXd const& target_phat,
                                 std::vector<Eigen::MatrixXd> const& detected_obstacle_list_qhat,
                                 float wind_direction,
                                 int only_direction_mode,
                                 int have_to_avoid_obstacle,
                                 float step_coeff,
                                 float direction_boat_obstacle,
                                 MatrixXd& potential_Z);

    void avoidObstacle(int& can_compute_a_new_avoidance_point,
                       std::vector<Eigen::MatrixXd>& line_to_follow,
                       MatrixXd const& collisioned_Obstacle,
                       MatrixXd& avoidance_collision_point,
                       std::vector<Eigen::MatrixXd>& detected_obstacle_list_qhat,
                       MatrixXd const& target_phat,
                       MatrixXd const& boat_state,
                       MatrixXd const& point_x,
                       MatrixXd const& point_y,
                       float wind_direction,
                       float radius_obstacle,
                       float radius_corridor,
                       float dim,
                       int only_direction_mode,
                       int have_to_avoid_obstacle,
                       float step_coeff,
                       float direction_boat_obstacle);

    void obstacleOnACollisionCourse(MatrixXd const& boat_state,
                                    std::vector<Eigen::MatrixXd> const& detected_obstacle_list_qhat,
                                    float radius_obstacle,
                                    int can_compute_a_new_avoidance_point);
    MatrixXd createWall(MatrixXd const& starting_point,MatrixXd const& ending_point,float step);


*/

};

#endif
