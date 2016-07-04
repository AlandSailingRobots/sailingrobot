#ifndef __COLLISIONAVOIDANCEBEHAVE_H__
#define __COLLISIONAVOIDANCEBEHAVE_H__
#include <stdint.h> //uint8_t
#include <math.h>
#include <algorithm>
#include <cmath>
#include "coursecalculation/CourseCalculation.h"
#include "behavingclass.h"
#include "utility/Utility.h"
#include "waypointrouting/WaypointRouting.h"
#include "behavingclass/LineFollowBehave.h"
#include <Eigen/Dense>
using Eigen::MatrixXd;

class CollisionAvoidanceBehave:public BehavingClass{
public:
    CollisionAvoidanceBehave(DBHandler *db);
    ~CollisionAvoidanceBehave(){};

    bool init();

    bool computeCommands(SystemStateModel &m_systemStateModel,std::unique_ptr<Position> const& position,
                                      std::vector<float> &twdBuffer,
                                      const unsigned int twdBufferMaxSize,bool m_mockPosition,
                                      bool m_getHeadingFromCompass);  //make the computation of the commands return true if the computation was successfull

    double getRudderCommand();
    double getSailCommand();
    void manageDatabase(std::vector<float> &twdBuffer,SystemStateModel &m_systemStateModel);

private:
    bool m_tack;
    int m_wayPointCount;
    double distanceToNextWaypoint, boardingToNextWaypoint;
    double desiredHeading, desiredHeadingTackMode;
    double m_rudderCommand, m_sailCommand;
    double m_maxCommandAngle, m_maxSailAngle, m_minSailAngle;
    double m_sailAngle, m_rudderAngle, m_tackAngle;
    int m_tackingDirection;
    Commands m_commandHandler;
    WaypointModel m_previousWaypointModel;
    WaypointModel m_nextWaypointModel;
	CourseMath m_courseMath;
    std::vector<WaypointModel> m_ListOfWaypointModel;
    void setupWaypoints();
    void setNextWaypoint();
    void setPreviousWayPoint(SystemStateModel &m_systemStateModel);
    int getHeading(SystemStateModel &m_systemStateModel,bool m_mockPosition,bool m_getHeadingFromCompass,std::unique_ptr<Position> const& position);

    MatrixXd potential_Z;
    std::vector<MatrixXd> detected_obstacle_list_qhat;
    std::vector<MatrixXd> mock_obstacle_list;
    std::vector<MatrixXd> target_list;
    MatrixXd target_phat;

    MatrixXd createWall(MatrixXd const& starting_point,MatrixXd const& ending_point,float step);
    void obstacleOnACollisionCourse(MatrixXd const& boat_state, MatrixXd const& detected_obstacle_list_qhat);
    void mockObstacleDetection(MatrixXd const& boat_state, MatrixXd const& mock_obstacle_list, MatrixXd const& detected_obstacle_list_qhat);

    void calculatePotentialField(MatrixXd const& point_x,MatrixXd const& point_y,
                                MatrixXd const& boat_state,MatrixXd const& target_phat,
                                MatrixXd const& detected_obstacle_list_qhat,
                                MatrixXd& potential_Z);
    void calculateAvoidancePoint(MatrixXd const& boat_state, MatrixXd const& target_phat, MatrixXd const& detected_obstacle_list_qhat,MatrixXd const& potential_Z, MatrixXd& avoidance_point);
};

#endif
