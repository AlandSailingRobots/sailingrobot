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
    m_tackAngle = 0.872665; //50°
    desiredHeadingTackMode = 0;
}

bool LineFollowBehaviour::init()
{                                                        
    m_wayPointCount = 0;                                                                 //|   If changed back to LineFollow during run, 
    m_previousWaypointModel.id = "";                                                     //|   reset values.

    printf("Setting up next Waypoint\n");
	setNextWaypoint(m_nextWaypointModel);
    printf("Setting up: OK\n");

    Logger::info("LineFollow Behaviour started");
    Logger::info("Waypoint target - ID: %s, Lon: %f, Lat: %f",  m_nextWaypointModel.id.c_str(), 
                                                                m_nextWaypointModel.positionModel.longitude, 
                                                                m_nextWaypointModel.positionModel.latitude);
    return true;
}

double LineFollowBehaviour::calculateSignedDistance(std::unique_ptr<Position> const& position)
{
    int earthRadius = 6371000;
    
    std::array<double, 3> prevWPCoord = //a
     {  earthRadius * cos(Utility::degreeToRadian(m_previousWaypointModel.positionModel.latitude)) * cos(Utility::degreeToRadian(m_previousWaypointModel.positionModel.longitude)),
        earthRadius * cos(Utility::degreeToRadian(m_previousWaypointModel.positionModel.latitude)) * sin(Utility::degreeToRadian(m_previousWaypointModel.positionModel.longitude)),
        earthRadius * sin(Utility::degreeToRadian(m_previousWaypointModel.positionModel.latitude))};
    std::array<double, 3> nextWPCoord = //b
     {  earthRadius * cos(Utility::degreeToRadian(m_nextWaypointModel.positionModel.latitude)) * cos(Utility::degreeToRadian(m_nextWaypointModel.positionModel.longitude)),
        earthRadius * cos(Utility::degreeToRadian(m_nextWaypointModel.positionModel.latitude)) * sin(Utility::degreeToRadian(m_nextWaypointModel.positionModel.longitude)),
        earthRadius * sin(Utility::degreeToRadian(m_nextWaypointModel.positionModel.latitude))};
        std::array<double, 3> boatCoord = //m
     {  earthRadius * cos(Utility::degreeToRadian(position->getModel().latitude)) * cos(Utility::degreeToRadian(position->getModel().longitude)),
        earthRadius * cos(Utility::degreeToRadian(position->getModel().latitude)) * sin(Utility::degreeToRadian(position->getModel().longitude)),
        earthRadius * sin(Utility::degreeToRadian(position->getModel().latitude))};
    
    double normAB = sqrt(prevWPCoord[0]*prevWPCoord[0] + prevWPCoord[1]*prevWPCoord[1] + prevWPCoord[2]*prevWPCoord[2])   * //||a|| 
                       sqrt(nextWPCoord[0]*nextWPCoord[0] + nextWPCoord[1]*nextWPCoord[1] + nextWPCoord[2]*nextWPCoord[2]); //||b||
                
    std::array<double, 3> oab = //vector normal to plane
    {   (prevWPCoord[1]*nextWPCoord[2] - prevWPCoord[2]*nextWPCoord[1]) / normAB,       //Vector product: A^B divided by norm a * norm b        a^b / ||a|| ||b||
        (prevWPCoord[2]*nextWPCoord[0] - prevWPCoord[0]*nextWPCoord[2]) / normAB,
        (prevWPCoord[0]*nextWPCoord[1] - prevWPCoord[1]*nextWPCoord[0]) / normAB };

    double signedDistance = boatCoord[0]*oab[0] + boatCoord[1]*oab[1] + boatCoord[2]*oab[2]; 

    return signedDistance;
}

double LineFollowBehaviour::calculateAngleOfDesiredTrajectory(std::unique_ptr<Position> const& position)
{
    int earthRadius = 6371000;

    std::array<double, 3> prevWPCoord =
     {  earthRadius * cos(Utility::degreeToRadian(m_previousWaypointModel.positionModel.latitude)) * cos(Utility::degreeToRadian(m_previousWaypointModel.positionModel.longitude)),
        earthRadius * cos(Utility::degreeToRadian(m_previousWaypointModel.positionModel.latitude)) * sin(Utility::degreeToRadian(m_previousWaypointModel.positionModel.longitude)),
        earthRadius * sin(Utility::degreeToRadian(m_previousWaypointModel.positionModel.latitude))};
    std::array<double, 3> nextWPCoord =
     {  earthRadius * cos(Utility::degreeToRadian(m_nextWaypointModel.positionModel.latitude)) * cos(Utility::degreeToRadian(m_nextWaypointModel.positionModel.longitude)),
        earthRadius * cos(Utility::degreeToRadian(m_nextWaypointModel.positionModel.latitude)) * sin(Utility::degreeToRadian(m_nextWaypointModel.positionModel.longitude)),
        earthRadius * sin(Utility::degreeToRadian(m_nextWaypointModel.positionModel.latitude))};

    double M[2][3] = 
    {   {-sin(Utility::degreeToRadian(position->getModel().longitude)), cos(Utility::degreeToRadian(position->getModel().longitude)), 0},
        {-cos(Utility::degreeToRadian(position->getModel().longitude))*sin(Utility::degreeToRadian(position->getModel().latitude)),
         -sin(Utility::degreeToRadian(position->getModel().longitude))*sin(Utility::degreeToRadian(position->getModel().latitude)), 
         cos(Utility::degreeToRadian(position->getModel().latitude))}};

    std::array<double, 3> bMinusA = { nextWPCoord[0]-prevWPCoord[0], nextWPCoord[1]-prevWPCoord[1], nextWPCoord[2]-prevWPCoord[2]};

                        // 2x3 * 1x3
    double phi = atan2(M[0][0]*bMinusA[0] + M[0][1]*bMinusA[1] + M[0][2]*bMinusA[2],   M[1][0]*bMinusA[0] + M[1][1]*bMinusA[1] + M[1][2]*bMinusA[2]);

    return phi;
}

void LineFollowBehaviour::computeCommands(SystemStateModel &systemStateModel,std::unique_ptr<Position> const& position,
                                      double trueWindDirection, bool mockPosition, bool getHeadingFromCompass){
    
    trueWindDirection = Utility::degreeToRadian(trueWindDirection);

    if(waypointsChanged) //if waypoints changed during run, check to see if current targeted waypoints have changed
    {
        int temp = m_wayPointCount;
        m_wayPointCount = 0; //set 0 so it doesn't set previous to next.
        setPreviousWayPoint(systemStateModel);
        setNextWaypoint(m_nextWaypointModel);
        waypointsChanged = false;
        m_wayPointCount = temp;
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
        if(m_previousWaypointModel.id == "")
            setPreviousWayPoint(systemStateModel);

        //GET DIRECTION - From the book Robotic Sailing 2012 and Robotic Sailing 2015
        currentHeading = Utility::degreeToRadian(getHeading(systemStateModel,mockPosition,getHeadingFromCompass,position, m_nextWaypointModel));
        double signedDistance = calculateSignedDistance(position); // 'e'
        int maxTackDistance = 20; //'r'
        double phi = calculateAngleOfDesiredTrajectory(position);
        double desiredHeading = phi - (2 * (M_PI / 4)/M_PI) * atan2(signedDistance,maxTackDistance);
        desiredHeading = Utility::limitRadianAngleRange(desiredHeading);
        //CHECK IF TACKING IS NEEDED
        if(abs(signedDistance) > maxTackDistance)
            m_tackingDirection = Utility::sgn(signedDistance);

        if( (cos(trueWindDirection - desiredHeading) + cos(m_tackAngle) < 0) || (cos(trueWindDirection - phi) + cos(m_tackAngle) < 0))
        {
            if(!m_tack){
                m_tackingDirection = Utility::sgn(currentHeading-(fmod(trueWindDirection, M_PI) - M_PI));
                m_tack = true;
            }
            desiredHeading = M_PI + trueWindDirection - m_tackingDirection * m_tackAngle;
            desiredHeading = Utility::limitRadianAngleRange(desiredHeading);

        }
// else if( (cos(trueWindDirection + M_PI - desiredHeading_star) + cos(0) < 0) ||  //Check if boat direction is same as truewind. NOT TESTED
        //              ( (abs(signedDistance) < maxTackDistance) && (cos(trueWindDirection + M_PI - phi) + cos(0) < 0) ) )
        // {
        //     if(!m_tack)              //NOT TESTED, TRY AT ANOTHER STAGE. SIMONS CODE
        //     {
        //         m_tackingDirection = Utility::sgn(currentHeading-(fmod(trueWindDirection + M_PI, M_PI) - M_PI));
        //         m_tack = true;
        //     }
        //     desiredHeading = trueWindDirection - m_tackingDirection * m_tackAngle;
// }
        else
            m_tack = false;

// if(abs(signedDistance) > maxTackDistance) //OLD VERSION, OUTCOMMENTED TO TEST SIMONS NEW ^
        // {
        //     m_tackingDirection = Utility::sgn(signedDistance);
        //     m_tack = false;
        // }
        //To stop the boat from oscillating during tack, we set the boat to tack from one maxTackDistance of the line to the other, not letting it switch direction in between.
        // double desiredHeading2 = desiredHeading; //Save desiredHeading into a temp variable so we don't have faulty values for the if-statements
        // if(cos(trueWindDirection - desiredHeading) + cos(m_tackAngle + M_PI/8) > 0)
        //     m_tack = false;
        // else if(m_tack)
        //     desiredHeading2 = desiredHeadingTackMode;
        // else if(cos(trueWindDirection - desiredHeading) + cos(m_tackAngle) < 0)                 
        // {
        //     m_tack = true;
        //     desiredHeading2 = M_PI + trueWindDirection - m_tackAngle * m_tackingDirection;               
        //     desiredHeadingTackMode = desiredHeading2;
        // }

        // if(abs(signedDistance) > maxTackDistance)
        //     desiredHeading2 = M_PI + trueWindDirection - m_tackAngle * m_tackingDirection;
// desiredHeading = desiredHeading2; //set the value back from the temp variable //END OF OLD VERSION^

        //SET RUDDER
        if(cos(currentHeading - desiredHeading) < 0) //if boat is going the wrong direction
            m_rudderCommand = Utility::sgn(systemStateModel.gpsModel.speed) * m_maxCommandAngle * Utility::sgn(sin(currentHeading - desiredHeading));
        else                      
            m_rudderCommand = Utility::sgn(systemStateModel.gpsModel.speed) * m_maxCommandAngle * sin(currentHeading - desiredHeading);
       
        //SET SAIL
        double apparentWindDirection = Utility::getApparentWindDirection(systemStateModel, currentHeading, trueWindDirection);
        m_sailCommand = -Utility::sgn(apparentWindDirection) * ( ((m_minSailAngle - m_maxSailAngle) / M_PI) * abs(apparentWindDirection) + m_maxSailAngle);


        printf("CurrentHeading: %f       signedDistance: %f        Phi: %f        Desired heading: %f \n", currentHeading, signedDistance, phi, desiredHeading);
        printf("bearingToNextWaypoint: %f\n", Utility::degreeToRadian(bearingToNextWaypoint));
        printf("Speed: %f      RudderCommand: %f     SailCommand: %f       TrueWindDirection: %f \n", systemStateModel.gpsModel.speed, m_rudderCommand, m_sailCommand, trueWindDirection);
        printf("Tacking: %d     TackingDirection: %d\n", m_tack, m_tackingDirection);

    } else {
        Logger::warning("%s gps NaN. Using values from last iteration", __PRETTY_FUNCTION__);
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
    currentHeading,
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
            m_previousWaypointModel.id = '0';
            printf("Set m_previousWaypointModel to boat position\n");
        }
        else 
            m_previousWaypointModel = waypointModel;
    }
    else if(m_wayPointCount > 0) //if waypoints passed, set previous waypoint to the one recently passed
        m_previousWaypointModel = m_nextWaypointModel;

    std::cout << "Previous waypoint picked! ID:" << m_previousWaypointModel.id <<" lon: "
    << m_previousWaypointModel.positionModel.longitude
    << " lat: " << m_previousWaypointModel.positionModel.latitude << " rad: "
    << m_previousWaypointModel.radius << std::endl;
}

bool LineFollowBehaviour::getGoingStarboard()
{
    if(m_tackingDirection == 1) return true;
    else return false;
}