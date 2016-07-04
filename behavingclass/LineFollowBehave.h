#ifndef __LINEFOLLOWBEHAVE_H__
#define __LINEFOLLOWBEHAVE_H__
#include <stdint.h> //uint8_t

#include <math.h>
#include <algorithm>
#include <cmath>
#include "coursecalculation/CourseCalculation.h"
#include "behavingclass.h"
#include "utility/Utility.h"
#include "waypointrouting/WaypointRouting.h"
//#include "models/WaypointModel.h"

class LineFollowBehave:public BehavingClass{
public:
    LineFollowBehave(DBHandler *db);
    ~LineFollowBehave(){};

    bool init();

    bool computeCommands(SystemStateModel &m_systemStateModel,std::unique_ptr<Position> const& position,
                                      std::vector<float> &twdBuffer,
                                      const unsigned int twdBufferMaxSize,bool m_mockPosition,
                                      bool m_getHeadingFromCompass);  //make the computation of the commands return true if the computation was successfull

    double getRudderCommand();                
    double getSailCommand();
    void manageDatabase(std::vector<float> &twdBuffer,SystemStateModel &m_systemStateModel);
    double calculateSignedDistance(std::unique_ptr<Position> const& position);
    double calculateAngleOfDesiredTrajectory(std::unique_ptr<Position> const& position);

private:
    bool m_tack; 
    int m_wayPointCount;
    double distanceToNextWaypoint, bearingToNextWaypoint;
    double desiredHeading, desiredHeadingTackMode;
    double m_rudderCommand, m_sailCommand;
    double m_maxCommandAngle, m_maxSailAngle, m_minSailAngle;
    double m_sailAngle, m_rudderAngle, m_tackAngle;
    int m_tackingDirection;
    Commands m_commandHandler;
    WaypointModel m_previousWaypointModel;
    WaypointModel m_nextWaypointModel;
	CourseMath m_courseMath;
    void setNextWaypoint();
    void setPreviousWayPoint(SystemStateModel &m_systemStateModel);
    bool getGoingStarboard();
    int getHeading(SystemStateModel &m_systemStateModel,bool m_mockPosition,bool m_getHeadingFromCompass,std::unique_ptr<Position> const& position);
};

#endif