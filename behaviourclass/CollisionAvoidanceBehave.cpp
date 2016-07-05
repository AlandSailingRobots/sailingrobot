#include "CollisionAvoidanceBehave.h"
CollisionAvoidanceBehave::CollisionAvoidanceBehave(DBHandler *db):
  BehavingClass(db),//super class call
  m_previousWaypointModel(PositionModel(1.5,2.7), 100, "", 6),
  m_nextWaypointModel(PositionModel(1.5,2.7), 100, "", 6)
{
    m_wayPointCount = 0;
    m_tackingDirection = 1;
    m_maxCommandAngle = M_PI / 6;
    m_maxSailAngle = M_PI / 32;
    m_minSailAngle = M_PI / 5.2f;
    m_tackAngle = M_PI / 3;
    desiredHeadingTackMode = 0;
}

bool CollisionAvoidanceBehave::init()
{
    printf("Entered init...\n");
    while(m_ListOfWaypointModel.size() > 0)                                                  //|
        m_ListOfWaypointModel.clear();                                                       //| If changed back to LineFollow during run,
    m_wayPointCount = 0;                                                                     //| reset values.
    m_previousWaypointModel.id = "";                                                     //|

    printf("Setting up Waypoints\n");
	setupWaypoints();
    printf("OK\n");

    printf("*SailingRobot::run() LineFollow started.\n");
    std::cout << "Waypoint target - ID: " << m_nextWaypointModel.id << " lon: " <<
    m_nextWaypointModel.positionModel.longitude	<< " lat : " <<
    m_nextWaypointModel.positionModel.latitude << std::endl;

    return true;
}

double CollisionAvoidanceBehave::getSailCommand(){
  return m_sailCommand;
}

double CollisionAvoidanceBehave::getRudderCommand(){
  return m_rudderCommand;
}


bool CollisionAvoidanceBehave::computeCommands(SystemStateModel &m_systemStateModel,std::unique_ptr<Position> const& position,
                                    std::vector<float> &twdBuffer,
                                    const unsigned int twdBufferMaxSize,bool m_mockPosition,bool m_getHeadingFromCompass){
/**
    if(m_previousWaypointModel.id == "")
        setPreviousWayPoint(m_systemStateModel);

    position->updatePosition();

    double trueWindDirection = Utility::meanOfAngles(twdBuffer);
    boardingToNextWaypoint = m_courseMath.calculateBTW(position->getModel(), m_nextWaypointModel.positionModel); //calculated for database
    distanceToNextWaypoint = m_courseMath.calculateDTW(position->getModel(), m_nextWaypointModel.positionModel);
    if(distanceToNextWaypoint < m_nextWaypointModel.radius)
    {
        //When arrive at next waypoint
        std::cout << "Waypoint reached!" << std::endl;
        m_wayPointCount++;
        setNextWaypoint();
        setPreviousWayPoint(m_systemStateModel);
    }

    //GOTTA CHECK IF GPS IS ONLINE
    if (m_systemStateModel.gpsModel.online) {
        //GET DIRECTION - From the book Robotic Sailing 2012 and Robotic Sailing 2015
        double currentHeading = getHeading(m_systemStateModel,m_mockPosition,m_getHeadingFromCompass,position);
        double signedDistance = calculateSignedDistance(position); // 'e'
        int maxTackDistance = 10; //'r'
        double phi = calculateAngleOfDesiredTrajectory(position);
        desiredHeading = phi - (2 * (M_PI / 4)/M_PI) * atan2(signedDistance,maxTackDistance);

        //CHECK IF TACKING IS NEEDED line 128 - 149.
        if(abs(signedDistance) > maxTackDistance)
        {
            m_tackingDirection = Utility::sgn(signedDistance);
            m_tack = false;
        }

        //To stop the boat from oscillating during tack, we set the boat to tack from one maxTackDistance of the line to the other, not letting it switch direction in between.
        double desiredHeading2 = desiredHeading; //Save desiredHeading into a temp variable so we don't have faulty values for the if-statements
        if(cos(trueWindDirection - desiredHeading) + cos(m_tackAngle + M_PI/8) > 0)
            m_tack = false;
        else if(m_tack)
            desiredHeading2 = desiredHeadingTackMode;
        else if(cos(trueWindDirection - desiredHeading) + cos(m_tackAngle) < 0)
        {
            m_tack = true;
            desiredHeading2 = M_PI + trueWindDirection - m_tackAngle * m_tackingDirection;
            desiredHeadingTackMode = desiredHeading2;
        }

        if(abs(signedDistance) > maxTackDistance)
            desiredHeading2 = M_PI + trueWindDirection - m_tackAngle * m_tackingDirection;
        desiredHeading = desiredHeading2; //set the value back from the temp variable


        //SET RUDDER
        if(cos(currentHeading - desiredHeading) < 0) //if boat is going the wrong direction
            m_rudderAngle = Utility::sgn(m_systemStateModel.gpsModel.speed) * m_maxCommandAngle * Utility::sgn(sin(currentHeading - desiredHeading));
        else
            m_rudderAngle = Utility::sgn(m_systemStateModel.gpsModel.speed) * m_maxCommandAngle * sin(currentHeading - desiredHeading);
        //m_rudderCommand = m_commandHandler.rudderCommand(m_rudderAngle, desiredHeading, m_maxCommandAngle); //Unsure about how this should be...
        m_rudderCommand = m_rudderAngle; //Is the angle already perfectly fixed or should I do as the line above??!?!

        //SET SAIL                          //Calculate apparent wind from true wind.
        std::array<double, 2> wcaw = { m_systemStateModel.windsensorModel.speed * cos(trueWindDirection - currentHeading) - m_systemStateModel.gpsModel.speed,
                                        m_systemStateModel.windsensorModel.speed * sin(trueWindDirection - currentHeading)};
        double apparentWindAngle = atan2(wcaw[0], wcaw[1]);

        m_sailAngle = -Utility::sgn(apparentWindAngle) * ( ((m_minSailAngle - m_maxSailAngle) / M_PI) * abs(apparentWindAngle) + m_maxSailAngle);
        //m_sailCommand = m_commandHandler.sailCommand(m_sailAngle);
        m_sailCommand = m_sailAngle;

    } else {
        m_logger.error("SailingRobot::run(), gps NaN. Using values from last iteration.");
    }
*/
    return true;
}


void CollisionAvoidanceBehave::manageDatabase(std::vector<float> &twdBuffer,SystemStateModel &m_systemStateModel){
  //logging
  /*
  bool routeStarted = false;
  m_dbHandler->insertDataLog(
    m_systemStateModel,
    0,
    0,
    distanceToNextWaypoint,
    boardingToNextWaypoint,
    desiredHeading,
    m_tack,
    getGoingStarboard(),
    atoi(m_nextWaypointModel.id.c_str()),
    Utility::meanOfAngles(twdBuffer),
    routeStarted
  );
  */
}


void CollisionAvoidanceBehave::setupWaypoints(){
    /*
    m_dbHandler->getWaypointFromTable(m_nextWaypointModel);
    printf("first set");
    m_dbHandler->getUnharvestedWaypointsFromTable(m_ListOfWaypointModel); //Adds all unharvested waypoints into m_ListOfWaypointModel
    if(m_ListOfWaypointModel.size() > 0)
        m_nextWaypointModel = m_ListOfWaypointModel[m_wayPointCount]; // m_wayPointCount = 0 at start
    else
    printf("no waypoints in list\n");
    printf(m_nextWaypointModel.id.c_str());
}

void CollisionAvoidanceBehave::setNextWaypoint(){
    if((unsigned)m_wayPointCount < m_ListOfWaypointModel.size())
        m_nextWaypointModel = m_ListOfWaypointModel[m_wayPointCount];
}

void CollisionAvoidanceBehave::setPreviousWayPoint(SystemStateModel &m_systemStateModel)
{
    if(m_wayPointCount == 0) //if no waypoints have been passed yet
    {
        //Check list if any waypoints have been passed earlier (incase hardreset has been made earlier, resulting in wayPointCount resetting)
        WaypointModel waypointModel = m_dbHandler->getPreviouslyHarvestedWaypoint();

        if(waypointModel.id == ""){//If no waypoints had been harvested, set previouspoint to boats startingposition
            m_previousWaypointModel.positionModel.longitude = m_systemStateModel.gpsModel.positionModel.longitude;
            m_previousWaypointModel.positionModel.latitude = m_systemStateModel.gpsModel.positionModel.latitude;
        }
        else
            m_previousWaypointModel = waypointModel;
    }
    else if(m_wayPointCount > 0) //if waypoints passed, set previous waypoint to the one recently passed
        m_previousWaypointModel = m_nextWaypointModel;
*/
}

int CollisionAvoidanceBehave::getHeading(SystemStateModel &m_systemStateModel,bool m_mockPosition,bool m_getHeadingFromCompass,std::unique_ptr<Position> const& position) {

	int useGpsForHeadingKnotSpeed = 1;
	bool acceptedCompassSpeed = Utility::directionAdjustedSpeed(m_systemStateModel.gpsModel.heading, m_systemStateModel.compassModel.heading, m_systemStateModel.gpsModel.speed) < useGpsForHeadingKnotSpeed;

    if(m_mockPosition) {
        return position->getHeading();
    }

    //only accept getHeadingFromCompass if speed less than 1
	if (m_getHeadingFromCompass && acceptedCompassSpeed) {
    	return Utility::addDeclinationToHeading(m_systemStateModel.compassModel.heading, m_nextWaypointModel.declination);
	}
    return m_systemStateModel.gpsModel.heading;

}
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
