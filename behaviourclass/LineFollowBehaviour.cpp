 

/****************************************************************************************
 *
 * File:
 * 		LineFollowBehaviour.cpp
 *
 * Purpose:
 *		This class LineFollowBehaviour compute the commands of the boat in order to follow
 *    lines given by the waypoints.
 *
 * Developer Notes: algorithm inspired and modified from Luc Jaulin and
 *    Fabrice Le Bars  "An Experimental Validation of a Robust Controller with the VAIMOS
 *    Autonomous Sailboat" and "Modeling and Control for an Autonomous Sailboat: A
 *    Case Study" from Jon Melin, Kjell Dahl and Matia Waller
 *
 *
 ***************************************************************************************/


#include <math.h>
#include <algorithm>
#include <cmath>

#include "LineFollowBehaviour.h"


LineFollowBehaviour::LineFollowBehaviour(DBHandler *db):
  RoutingBehaviour(db),//super class call
  m_previousWaypointModel(PositionModel(0,0), 0, "", 0),
  m_nextWaypointModel(PositionModel(0,0), 0, "", 0)
{
    m_wayPointCount = 0;
    m_tackingDirection = 1;
    m_maxCommandAngle = M_PI / 6;
    m_maxSailAngle = M_PI / 4.2f;
    m_minSailAngle = M_PI / 32;
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

double LineFollowBehaviour::calculateSignedDistanceToLine(std::unique_ptr<Position> const& position,float &afterWaypoint)
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

    std::array<double, 3> oab = //vector normal to plane
    {   (prevWPCoord[1]*nextWPCoord[2] - prevWPCoord[2]*nextWPCoord[1]),       //Vector product: A^B divided by norm ||a^b||     a^b / ||a^b||
        (prevWPCoord[2]*nextWPCoord[0] - prevWPCoord[0]*nextWPCoord[2]),
        (prevWPCoord[0]*nextWPCoord[1] - prevWPCoord[1]*nextWPCoord[0])};

    double normOAB =  sqrt(pow(oab[0],2)+ pow(oab[1],2) + pow(oab[2],2));

    oab[0] = oab[0]/normOAB;
    oab[1] = oab[1]/normOAB;
    oab[2] = oab[2]/normOAB;

    double signedDistance = boatCoord[0]*oab[0] + boatCoord[1]*oab[1] + boatCoord[2]*oab[2];

    //compute if boat is after waypointModel
    std::array<double, 3> orthogonal_to_AB_from_B = //C the point such as  BC is orthogonal to AB
    {  nextWPCoord[0]+oab[0],
       nextWPCoord[1]+oab[1],
       nextWPCoord[2]+oab[2]
    };

    std::array<double, 3> obc = //vector normal to plane
    {   (orthogonal_to_AB_from_B[1]*nextWPCoord[2] - orthogonal_to_AB_from_B[2]*nextWPCoord[1]) ,       //Vector product: C^B divided by norm ||c^b||     c^b / ||c^b||
        (orthogonal_to_AB_from_B[2]*nextWPCoord[0] - orthogonal_to_AB_from_B[0]*nextWPCoord[2]) ,
        (orthogonal_to_AB_from_B[0]*nextWPCoord[1] - orthogonal_to_AB_from_B[1]*nextWPCoord[0])};

    double normOBC =  sqrt(pow(obc[0],2)+ pow(obc[1],2) + pow(obc[2],2));


    //float temp = boatCoord[0]*obc[0] + boatCoord[1]*obc[1] + boatCoord[2]*obc[2];
    afterWaypoint = boatCoord[0]*obc[0]/normOBC + boatCoord[1]*obc[1]/normOBC + boatCoord[2]*obc[2]/normOBC;


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

    /* add pi because trueWindDirection is originally origin of wind but algorithm need direction*/
    trueWindDirection = Utility::degreeToRadian(trueWindDirection)+M_PI;

    if (not systemStateModel.gpsModel.online) //gps not online return before initialising
        return;

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

    float afterNextWaypoint;
    double signedDistance =     calculateSignedDistanceToLine(position,afterNextWaypoint); // 'e'

    /* Check to see if waypoint is reached or if boat has passed the orthogonal to the line
     * otherwise the boat will continue to follow old line if it passed the waypoint too far away
     */
    if(distanceToNextWaypoint < m_nextWaypointModel.radius || afterNextWaypoint > 0)
    {
        m_wayPointCount++;
        harvestWaypoint(m_nextWaypointModel);
        setPreviousWayPoint(systemStateModel);
        setNextWaypoint(m_nextWaypointModel);
    }

    if (systemStateModel.gpsModel.online) {
        if(m_previousWaypointModel.id == "")
            setPreviousWayPoint(systemStateModel);

        //GET DIRECTION - From the book Robotic Sailing 2012 and Robotic Sailing 2015
        currentHeading = Utility::degreeToRadian(getHeading(systemStateModel,mockPosition,getHeadingFromCompass,position, m_nextWaypointModel));

        int maxTackDistance = 20; //'r'
        double phi = calculateAngleOfDesiredTrajectory(position);
        double desiredHeading = phi + (2 * (M_PI / 4)/M_PI) * atan(signedDistance/maxTackDistance); //heading to smoothly join the line
        desiredHeading = Utility::limitRadianAngleRange(desiredHeading);

        //Change tacking direction when reaching max distance
        if(abs(signedDistance) > maxTackDistance)
            m_tackingDirection = -Utility::sgn(signedDistance);

        //Check if tacking is needed
        if( (cos(trueWindDirection - desiredHeading) + cos(m_tackAngle) < 0) || (cos(trueWindDirection - phi) + cos(m_tackAngle) < 0))
        {
            if(!m_tack){ /* initialize tacking direction */
                m_tackingDirection = -Utility::sgn(currentHeading-(fmod(trueWindDirection+M_PI, 2*M_PI) - M_PI));
                m_tack = true;
            }

            desiredHeading = M_PI + trueWindDirection - m_tackingDirection * m_tackAngle;/* sail around the wind direction */
            desiredHeading = Utility::limitRadianAngleRange(desiredHeading);
        }
        else
            m_tack = false;

        //SET RUDDER
        if(cos(currentHeading - desiredHeading) < 0) //if boat heading is too far away from desired heading
            m_rudderCommand = -Utility::sgn(systemStateModel.gpsModel.speed) * m_maxCommandAngle * Utility::sgn(sin(currentHeading - desiredHeading));
        else
            m_rudderCommand = -Utility::sgn(systemStateModel.gpsModel.speed) * m_maxCommandAngle * sin(currentHeading - desiredHeading);

        //SET SAIL
        double apparentWindDirection = Utility::getApparentWindDirection(systemStateModel, currentHeading, trueWindDirection)*M_PI/180;
        m_sailCommand = abs( ((m_minSailAngle - m_maxSailAngle) / M_PI) * abs(apparentWindDirection) + m_maxSailAngle);

        /*printf("CurrentHeading: %f       signedDistance: %f        Phi: %f        Desired heading: %f Diff heading: %f \n", currentHeading, signedDistance, phi, desiredHeading, desiredHeading-phi);
        printf("bearingToNextWaypoint: %f\n", Utility::degreeToRadian(bearingToNextWaypoint));
        printf("Speed: %f      RudderCommand: %f     SailCommand: %f       TrueWindDirection: %f \n", systemStateModel.gpsModel.speed, m_rudderCommand, m_sailCommand, trueWindDirection);
        printf("Tacking: %d     TackingDirection: %d Afterwaypoint: %.4f desiredDir %.4f\n", m_tack, m_tackingDirection,afterNextWaypoint,(2 * (M_PI / 4)/M_PI) * atan(signedDistance/maxTackDistance));
        */
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
        if(not m_dbHandler->getWaypointFromTable(m_previousWaypointModel, true))
        {//If no waypoints had been harvested, set previouspoint to boats startingposition
            m_previousWaypointModel.positionModel.longitude = systemStateModel.gpsModel.positionModel.longitude;
            m_previousWaypointModel.positionModel.latitude = systemStateModel.gpsModel.positionModel.latitude;
            m_previousWaypointModel.id = '0';
            printf("Set m_previousWaypointModel to boat position\n");
        }
        //if true sets m_previousWaypointModel to latest harvested waypoint.
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
