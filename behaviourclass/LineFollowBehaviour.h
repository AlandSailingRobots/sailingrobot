#ifndef __LINEFOLLOWBEHAVE_H__
#define __LINEFOLLOWBEHAVE_H__
#include <stdint.h> //uint8_t

#include <math.h>
#include <algorithm>
#include <cmath>
#include "coursecalculation/CourseCalculation.h"
#include "RoutingBehaviour.h"
#include "waypointrouting/WaypointRouting.h"
//#include "models/WaypointModel.h"

class LineFollowBehaviour:public RoutingBehaviour{
public:
    LineFollowBehaviour(DBHandler *db);
    ~LineFollowBehaviour(){};

    bool init();

    bool computeCommands(SystemStateModel &systemStateModel,std::unique_ptr<Position> const& position,
                                      double trueWindDirection, bool mockPosition,
                                      bool getHeadingFromCompass);  //make the computation of the commands return true if the computation was successfull

    void manageDatabase(std::vector<float> &twdBuffer,SystemStateModel &systemStateModel);
    double calculateSignedDistance(std::unique_ptr<Position> const& position);
    double calculateAngleOfDesiredTrajectory(std::unique_ptr<Position> const& position);

private:
    bool m_tack; 
    int m_wayPointCount;
    double distanceToNextWaypoint, bearingToNextWaypoint;
    double desiredHeading, desiredHeadingTackMode;
    double m_maxCommandAngle, m_maxSailAngle, m_minSailAngle;
    double m_tackAngle;
    int m_tackingDirection;
    Commands m_commandHandler;
    WaypointModel m_previousWaypointModel;
    WaypointModel m_nextWaypointModel;
	CourseMath m_courseMath;
    void setPreviousWayPoint(SystemStateModel &m_systemStateModel);
    bool getGoingStarboard();
};

#endif