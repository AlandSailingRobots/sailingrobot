#include "LineFollowBehaviour.h"


LineFollowBehaviour::LineFollowBehaviour(DBHandler *db):
  RoutingBehaviour(db),//super class call
  m_previousWaypointModel(PositionModel(0,0), 0, "", 0),
  m_nextWaypointModel(PositionModel(0,0), 0, "", 0)
{
    m_wayPointCount = 0;
    m_tackingDirection = 1;
    m_maxCommandAngle = M_PI / 6;
    m_maxSailAngle = M_PI / 32;
    m_minSailAngle = M_PI / 5.2f;
    m_tackAngle = M_PI / 3;
    desiredHeadingTackMode = 0;
}

bool LineFollowBehaviour::init()
{                                                        
    m_wayPointCount = 0;                                                                 //|   If changed back to LineFollow during run, 
    m_previousWaypointModel.id = "";                                                     //|   reset values.

    printf("Setting up next Waypoint\n");
	setNextWaypoint(m_nextWaypointModel);
    printf("Setting up: OK\n");

    printf("*SailingRobot::run() LineFollow started.\n");
    std::cout << "Waypoint target - ID: " << m_nextWaypointModel.id << " lon: " <<
    m_nextWaypointModel.positionModel.longitude	<< " lat : " <<
    m_nextWaypointModel.positionModel.latitude << std::endl;

    return true;
}


double LineFollowBehaviour::calculateSignedDistance(std::unique_ptr<Position> const& position)
{

    int earthRadius = 6371000;
    std::array<double, 3> a =
     {  earthRadius * Utility::degreeToRadian(cos(m_previousWaypointModel.positionModel.latitude)) * Utility::degreeToRadian(cos(m_previousWaypointModel.positionModel.longitude)),
        earthRadius * Utility::degreeToRadian(cos(m_previousWaypointModel.positionModel.latitude)) * Utility::degreeToRadian(cos(m_previousWaypointModel.positionModel.longitude)),
        earthRadius * Utility::degreeToRadian(sin(m_previousWaypointModel.positionModel.latitude))};
    std::array<double, 3> b =
     {  earthRadius * Utility::degreeToRadian(cos(m_nextWaypointModel.positionModel.latitude)) * Utility::degreeToRadian(cos(m_nextWaypointModel.positionModel.longitude)),
        earthRadius * Utility::degreeToRadian(cos(m_nextWaypointModel.positionModel.latitude)) * Utility::degreeToRadian(cos(m_nextWaypointModel.positionModel.longitude)),
        earthRadius * Utility::degreeToRadian(sin(m_nextWaypointModel.positionModel.latitude))};
    std::array<double, 3> m = 
    {   earthRadius * Utility::degreeToRadian(cos(position->getModel().latitude)) * Utility::degreeToRadian(cos(position->getModel().longitude)),
        earthRadius * Utility::degreeToRadian(cos(position->getModel().latitude)) * Utility::degreeToRadian(cos(position->getModel().longitude)),
        earthRadius * Utility::degreeToRadian(sin(position->getModel().latitude))};
                // a^b / ||a|| * ||b||
    double n = ( (a[1]*b[2] - a[2]*b[1]) + (a[2]*b[0] - a[0]*b[2]) + (a[0]*b[2] - a[1]*b[0]) ) /   //Vector product: A^B divided by
                ( sqrt(a[0]*a[0] + a[1]*a[1] + a[2]*a[2])   *   sqrt(b[0]*b[0] + b[1]*b[1] + b[2]*b[2])); // norm a * norm b

    double e = m[0]*n + m[1]*n + m[2]*n; //dot product       

    return e;
}

double LineFollowBehaviour::calculateAngleOfDesiredTrajectory(std::unique_ptr<Position> const& position)
{
    int earthRadius = 6371000;
    std::array<double, 3> a =
     {  earthRadius * Utility::degreeToRadian(cos(m_previousWaypointModel.positionModel.latitude)) * Utility::degreeToRadian(cos(m_previousWaypointModel.positionModel.longitude)),
        earthRadius * Utility::degreeToRadian(cos(m_previousWaypointModel.positionModel.latitude)) * Utility::degreeToRadian(cos(m_previousWaypointModel.positionModel.longitude)),
        earthRadius * Utility::degreeToRadian(sin(m_previousWaypointModel.positionModel.latitude))};
    std::array<double, 3> b =
     {  earthRadius * Utility::degreeToRadian(cos(m_nextWaypointModel.positionModel.latitude)) * Utility::degreeToRadian(cos(m_nextWaypointModel.positionModel.longitude)),
        earthRadius * Utility::degreeToRadian(cos(m_nextWaypointModel.positionModel.latitude)) * Utility::degreeToRadian(cos(m_nextWaypointModel.positionModel.longitude)),
        earthRadius * Utility::degreeToRadian(sin(m_nextWaypointModel.positionModel.latitude))};

    double M[2][3] = 
    {   {Utility::degreeToRadian(-sin(position->getModel().longitude)), Utility::degreeToRadian(cos(position->getModel().longitude)), 0},
        {Utility::degreeToRadian(-cos(position->getModel().longitude))*Utility::degreeToRadian(sin(position->getModel().latitude)),
         Utility::degreeToRadian(-sin(position->getModel().longitude))*Utility::degreeToRadian(sin(position->getModel().latitude)), 
         Utility::degreeToRadian(cos(position->getModel().latitude))}};

    std::array<double, 3> bMinusA = { b[0]-a[0], b[1]-a[1], b[2]-a[2]};

                        // 2x3 * 1x3
    double phi = atan2(M[0][0]*bMinusA[0] + M[0][1]*bMinusA[1] + M[0][2]*bMinusA[2],   M[1][0]*bMinusA[0] + M[1][1]*bMinusA[1] + M[1][2]*bMinusA[2]);

    return phi;
}

void LineFollowBehaviour::computeCommands(SystemStateModel &systemStateModel,std::unique_ptr<Position> const& position,
                                      double trueWindDirection, bool mockPosition, bool getHeadingFromCompass){

    if(m_previousWaypointModel.id == "")
        setPreviousWayPoint(systemStateModel);
    
    if(waypointsChanged)
    {
        m_wayPointCount = 0;
        setPreviousWayPoint(systemStateModel);
        setNextWaypoint(m_nextWaypointModel);
        waypointsChanged = false;
    }

    position->updatePosition();
    bearingToNextWaypoint = m_courseMath.calculateBTW(position->getModel(), m_nextWaypointModel.positionModel); //calculated for database
    distanceToNextWaypoint = m_courseMath.calculateDTW(position->getModel(), m_nextWaypointModel.positionModel);

    //Check to see if waypoint is reached
    if(distanceToNextWaypoint < m_nextWaypointModel.radius)
    {
        m_wayPointCount++;
        harvestWaypoint(m_nextWaypointModel);
        setPreviousWayPoint(systemStateModel);
        setNextWaypoint(m_nextWaypointModel);
    }

    //GOTTA CHECK IF GPS IS ONLINE
    if (systemStateModel.gpsModel.online) {
        //GET DIRECTION - From the book Robotic Sailing 2012 and Robotic Sailing 2015
        double currentHeading = getHeading(systemStateModel,mockPosition,getHeadingFromCompass,position, m_nextWaypointModel);
        double signedDistance = calculateSignedDistance(position); // 'e'
        int maxTackDistance = 10; //'r'
        double phi = calculateAngleOfDesiredTrajectory(position);
        desiredHeading = phi - (2 * (M_PI / 4)/M_PI) * atan2(signedDistance,maxTackDistance);

        //CHECK IF TACKING IS NEEDED
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
        {
            m_rudderCommand = Utility::sgn(systemStateModel.gpsModel.speed) * m_maxCommandAngle * Utility::sgn(sin(currentHeading - desiredHeading));
            std::cout << "                  Boat is going the       WRONG       direction" << std::endl;
        }
        else                      
        {
            m_rudderCommand = Utility::sgn(systemStateModel.gpsModel.speed) * m_maxCommandAngle * sin(currentHeading - desiredHeading);
                        std::cout << "                  Boat is going the       RIGHT        direction" << std::endl;
        }          

        //SET SAIL
        double apparentWindDirection = Utility::getApparentWindDirection(systemStateModel, currentHeading, trueWindDirection);
        m_sailCommand = -Utility::sgn(apparentWindDirection) * ( ((m_minSailAngle - m_maxSailAngle) / M_PI) * abs(apparentWindDirection) + m_maxSailAngle);

        std::cout << "speed: " << systemStateModel.gpsModel.speed << "   desiredHeading: " << desiredHeading << "   maxCommand: " << m_maxCommandAngle << 
        "   rudderCommand: " << m_rudderCommand  << "    SailCommand: " << m_sailCommand << std::endl;
        std::cout << "heading: " << currentHeading << std::endl;
        printf("Tacking: %d     TackingDirection: %d\n", m_tack, m_tackingDirection);
    } else {
        m_logger.error("SailingRobot::run(), gps NaN. Using values from last iteration.\n");
        printf("GPS not online\n");
    }
}


void LineFollowBehaviour::manageDatabase(double trueWindDirection, SystemStateModel &systemStateModel){
  //logging
  bool routeStarted = false;
  m_dbHandler->insertDataLog(
    systemStateModel,
    0,
    0,                                               
    distanceToNextWaypoint,
    bearingToNextWaypoint,
    desiredHeading,
    m_tack,
    getGoingStarboard(),
    atoi(m_nextWaypointModel.id.c_str()),
    trueWindDirection,
    routeStarted
  );
}

void LineFollowBehaviour::setPreviousWayPoint(SystemStateModel &systemStateModel)
{
    if(m_wayPointCount == 0) //if no waypoints have been passed yet
    {
        //Check list if any waypoints have been passed earlier (incase hardreset has been made earlier, resulting in wayPointCount resetting)
        WaypointModel waypointModel = m_dbHandler->getPreviouslyHarvestedWaypoint();
        
        if(waypointModel.id == ""){//If no waypoints had been harvested, set previouspoint to boats startingposition
            m_previousWaypointModel.positionModel.longitude = systemStateModel.gpsModel.positionModel.longitude;
            m_previousWaypointModel.positionModel.latitude = systemStateModel.gpsModel.positionModel.latitude;
        }
        else 
            m_previousWaypointModel = waypointModel;
    }
    else if(m_wayPointCount > 0) //if waypoints passed, set previous waypoint to the one recently passed
        m_previousWaypointModel = m_nextWaypointModel;
}

bool LineFollowBehaviour::getGoingStarboard()
{
    if(m_tackingDirection == 1) return true;
    else return false;
}