/****************************************************************************************
 *
 * File:
 * 		SailigLogicNewNode.h
 *
 * Purpose:
 *		Sailing Logic, makes boat sail to the right place with line following
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/

#include "SailingLogicNewNode.h"
#include "Messages/ActuatorPositionMsg.h"
#include "utility/Utility.h"
#include <math.h>
#include <algorithm>
#include <cmath>

#define DEFAULT_TWD_BUFFERSIZE 200

SailingLogicNewNode::SailingLogicNewNode(MessageBus& msgBus, DBHandler& db)
:  Node(NodeID::SailingLogic, msgBus), m_db(db),
    m_nextWaypointId(0),  
    m_nextWaypointLon(0), 
    m_nextWaypointLat(0),
    m_nextWaypointDeclination(0),
    m_nextWaypointRadius(0),
    m_prevWaypointId(0),
    m_prevWaypointLon(0), 
    m_prevWaypointLat(0),
    m_prevWaypointDeclination(0),
    m_prevWaypointRadius(0),
    m_waypointCount(0),
    m_tackingDirection(1)
{
    m_maxCommandAngle = M_PI / 6;
    m_maxSailAngle = M_PI / 32;
    m_minSailAngle = M_PI / 5.2f;
    m_tackAngle = 0.872665; //50°
}

bool SailingLogicNewNode::init()
{
    setupRudderCommand();
    setupSailCommand();
    twdBufferMaxSize = m_db.retrieveCellAsInt("buffer_config", "1", "true_wind");
	if(twdBufferMaxSize == 0)
		twdBufferMaxSize = DEFAULT_TWD_BUFFERSIZE;
    return true;
}

void SailingLogicNewNode::processMessage(const Message* msg)
{
    MessageType type = msg->messageType();

	switch(type)
	{
	case MessageType::VesselState:
		calculateActuatorPos((VesselStateMsg*)msg);
		break;
	case MessageType::WaypointData:
        {
            WaypointDataMsg* waypMsg = (WaypointDataMsg*)msg;
            m_nextWaypointId = waypMsg->nextId();
            m_nextWaypointLon = waypMsg->nextLongitude();
            m_nextWaypointLat = waypMsg->nextLatitude();
            m_nextWaypointDeclination = waypMsg->nextDeclination();
            m_nextWaypointRadius = waypMsg->nextRadius();
            setPrevWaypointData(waypMsg, (VesselStateMsg*)msg);
            
        }
		break;
	default:
		return;
	}
}

double SailingLogicNewNode::calculateSignedDistance(VesselStateMsg* msg)
{
    int earthRadius = 6371000;
    
    std::array<double, 3> prevWPCoord = //a
     {  earthRadius * cos(Utility::degreeToRadian(m_prevWaypointLat)) * cos(Utility::degreeToRadian(m_prevWaypointLon)),
        earthRadius * cos(Utility::degreeToRadian(m_prevWaypointLat)) * sin(Utility::degreeToRadian(m_prevWaypointLon)),
        earthRadius * sin(Utility::degreeToRadian(m_prevWaypointLat))};
    std::array<double, 3> nextWPCoord = //b
     {  earthRadius * cos(Utility::degreeToRadian(m_nextWaypointLat)) * cos(Utility::degreeToRadian(m_nextWaypointLon)),
        earthRadius * cos(Utility::degreeToRadian(m_nextWaypointLat)) * sin(Utility::degreeToRadian(m_nextWaypointLon)),
        earthRadius * sin(Utility::degreeToRadian(m_nextWaypointLat))};
        std::array<double, 3> boatCoord = //m
     {  earthRadius * cos(Utility::degreeToRadian(msg->latitude() )) * cos(Utility::degreeToRadian(msg->longitude() )),
        earthRadius * cos(Utility::degreeToRadian(msg->latitude() )) * sin(Utility::degreeToRadian(msg->longitude() )),
        earthRadius * sin(Utility::degreeToRadian(msg->latitude() ))};
    
    double normAB = sqrt(prevWPCoord[0]*prevWPCoord[0] + prevWPCoord[1]*prevWPCoord[1] + prevWPCoord[2]*prevWPCoord[2])   * //||a|| 
                       sqrt(nextWPCoord[0]*nextWPCoord[0] + nextWPCoord[1]*nextWPCoord[1] + nextWPCoord[2]*nextWPCoord[2]); //||b||
                
    std::array<double, 3> oab = //vector normal to plane
    {   (prevWPCoord[1]*nextWPCoord[2] - prevWPCoord[2]*nextWPCoord[1]) / normAB,       //Vector product: A^B divided by norm a * norm b        a^b / ||a|| ||b||
        (prevWPCoord[2]*nextWPCoord[0] - prevWPCoord[0]*nextWPCoord[2]) / normAB,
        (prevWPCoord[0]*nextWPCoord[1] - prevWPCoord[1]*nextWPCoord[0]) / normAB };

    double signedDistance = boatCoord[0]*oab[0] + boatCoord[1]*oab[1] + boatCoord[2]*oab[2]; 

    return signedDistance;
}

double SailingLogicNewNode::calculateAngleOfDesiredTrajectory(VesselStateMsg* msg)
{
    int earthRadius = 6371000;

    std::array<double, 3> prevWPCoord = 
     {  earthRadius * cos(Utility::degreeToRadian(m_prevWaypointLat)) * cos(Utility::degreeToRadian(m_prevWaypointLon)),
        earthRadius * cos(Utility::degreeToRadian(m_prevWaypointLat)) * sin(Utility::degreeToRadian(m_prevWaypointLon)),
        earthRadius * sin(Utility::degreeToRadian(m_prevWaypointLat))};
    std::array<double, 3> nextWPCoord = 
     {  earthRadius * cos(Utility::degreeToRadian(m_nextWaypointLat)) * cos(Utility::degreeToRadian(m_nextWaypointLon)),
        earthRadius * cos(Utility::degreeToRadian(m_nextWaypointLat)) * sin(Utility::degreeToRadian(m_nextWaypointLon)),
        earthRadius * sin(Utility::degreeToRadian(m_nextWaypointLat))};

    double M[2][3] = 
    {   {-sin(Utility::degreeToRadian(msg->longitude() )), cos(Utility::degreeToRadian(msg->longitude() )), 0},
        {-cos(Utility::degreeToRadian(msg->longitude() ))*sin(Utility::degreeToRadian(msg->latitude() )),
         -sin(Utility::degreeToRadian(msg->longitude() ))*sin(Utility::degreeToRadian(msg->latitude() )), 
         cos(Utility::degreeToRadian(msg->latitude() ))}};

    std::array<double, 3> bMinusA = { nextWPCoord[0]-prevWPCoord[0], nextWPCoord[1]-prevWPCoord[1], nextWPCoord[2]-prevWPCoord[2]};

                        // 2x3 * 1x3
    double phi = atan2(M[0][0]*bMinusA[0] + M[0][1]*bMinusA[1] + M[0][2]*bMinusA[2],   M[1][0]*bMinusA[0] + M[1][1]*bMinusA[1] + M[1][2]*bMinusA[2]);

    return phi;
}

void SailingLogicNewNode::calculateActuatorPos(VesselStateMsg* msg)
{
    double trueWindDirection = Utility::getTrueWindDirection(msg->windDir(), msg->windSpeed(), msg->speed(), msg->compassHeading(), twdBuffer, twdBufferMaxSize);

    //GET DIRECTION - From the book Robotic Sailing 2012 and Robotic Sailing 2015
    double currentHeading = getHeading(msg->gpsHeading(), msg->compassHeading(), msg->speed(), false, false);
    double signedDistance = calculateSignedDistance(msg); // 'e'
    int maxTackDistance = 20; //'r'
    double phi = calculateAngleOfDesiredTrajectory(msg);
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
    else
        m_tack = false;

    double rudderCommand, sailCommand;
    //SET RUDDER
    if(cos(currentHeading - desiredHeading) < 0) //if boat is going the wrong direction
        rudderCommand = Utility::sgn(msg->speed()) * m_maxCommandAngle * Utility::sgn(sin(currentHeading - desiredHeading));
    else                      
        rudderCommand = Utility::sgn(msg->speed()) * m_maxCommandAngle * sin(currentHeading - desiredHeading);
    
    //SET SAIL
    double apparentWindDirection = Utility::getApparentWindDirection(msg->windDir(), msg->windSpeed(), msg->speed(), currentHeading, trueWindDirection);
    sailCommand = -Utility::sgn(apparentWindDirection) * ( ((m_minSailAngle - m_maxSailAngle) / M_PI) * abs(apparentWindDirection) + m_maxSailAngle);

    rudderCommand = m_rudderCommand.getCommand(rudderCommand);
	sailCommand = m_sailCommand.getCommand(sailCommand);

    ActuatorPositionMsg *rudderMsg = new ActuatorPositionMsg(NodeID::RudderActuator, nodeID(), rudderCommand);
    ActuatorPositionMsg *sailMsg = new ActuatorPositionMsg(NodeID::SailActuator, nodeID(), sailCommand);
    m_MsgBus.sendMessage(rudderMsg);
    m_MsgBus.sendMessage(sailMsg);


    double bearingToNextWaypoint = m_courseMath.calculateBTW(msg->longitude(), msg->latitude(), m_nextWaypointLon, m_nextWaypointLat); //calculated for database
    double distanceToNextWaypoint = m_courseMath.calculateDTW(msg->longitude(), msg->latitude(), m_nextWaypointLon, m_nextWaypointLat);

    manageDatabase(msg, trueWindDirection, rudderCommand, sailCommand, currentHeading, distanceToNextWaypoint, bearingToNextWaypoint);
}

void SailingLogicNewNode::setPrevWaypointData(WaypointDataMsg* waypMsg, VesselStateMsg* vesselMsg)
{
    if(m_waypointCount == 0)
    {
        if(waypMsg->prevId() == 0) //Set previous waypoint to boat position
        {
            m_prevWaypointId = 0;
            m_prevWaypointLon = vesselMsg->longitude();
            m_prevWaypointLat = vesselMsg->latitude();
            m_prevWaypointDeclination = 0;
            m_prevWaypointRadius = 15;
        }
        else //Set previous waypoint to previously harvested waypoint
        {
            m_prevWaypointId = waypMsg->prevId();
            m_prevWaypointLon = waypMsg->prevLongitude();
            m_prevWaypointLat = waypMsg->prevLatitude();
            m_prevWaypointDeclination = waypMsg->prevDeclination();
            m_prevWaypointRadius = waypMsg->prevRadius();
        }
    }
    else //Set previous waypoint to next waypoint that was just reached
    {
        m_prevWaypointId = m_nextWaypointId;
        m_prevWaypointLon = m_nextWaypointLon;
        m_prevWaypointLat = m_nextWaypointLat;
        m_prevWaypointDeclination = m_nextWaypointDeclination;
        m_prevWaypointRadius = m_nextWaypointRadius;
    }
}

int SailingLogicNewNode::getHeading(int gpsHeading, int compassHeading, double gpsSpeed, bool mockPosition,bool getHeadingFromCompass) {
	//Use GPS for heading only if speed is higher than 1 knot
	int useGpsForHeadingKnotSpeed = 1;
	bool gpsForbidden = Utility::directionAdjustedSpeed(gpsHeading, compassHeading, gpsSpeed) < useGpsForHeadingKnotSpeed;

	getMergedHeading(gpsHeading, compassHeading, true); //decrease compass weight on each iteration

    // if(mockPosition) { //TODO - MOCK
    //     return position->getHeading(); //OUTCOMMENTED FOR NOW UNTIL WE FIGURE OUT MOCK
    // }

	if (getHeadingFromCompass) {
		//Should return compass heading if below one knot and not currently merging and vice versa
    	return Utility::addDeclinationToHeading(getMergedHeading(gpsHeading, compassHeading, gpsForbidden), m_nextWaypointDeclination);
	}
    return gpsHeading;
}

int SailingLogicNewNode::getMergedHeading(int gpsHeading, int compassHeading, bool increaseCompassWeight){
	//Shouldn't be hardcoded
	float tickRate = 0.01;

	int headingCompass = Utility::addDeclinationToHeading(compassHeading, m_nextWaypointDeclination);
	int headingGps = gpsHeading;

	if (increaseCompassWeight){
		m_gpsHeadingWeight = m_gpsHeadingWeight - tickRate; //Decrease gps weight
		if (m_gpsHeadingWeight < 0.0) m_gpsHeadingWeight = 0;
	}else{
		m_gpsHeadingWeight = m_gpsHeadingWeight + tickRate;
		if (m_gpsHeadingWeight > 1.0) m_gpsHeadingWeight = 1.0;
	}

	//Difference calculation
	float diff = ((headingGps - headingCompass) + 180 + 360);
	while (diff > 360) diff -= 360;
	diff -= 180;

	//Merge angle calculation
	int returnValue = 360 + headingCompass + (diff * m_gpsHeadingWeight);
	while (returnValue > 360) returnValue -= 360;

	return returnValue;
}

void SailingLogicNewNode::setupRudderCommand() 
{
	m_rudderCommand.setCommandValues(m_db.retrieveCellAsInt("rudder_command_config", "1","extreme_command"),
	        m_db.retrieveCellAsInt("rudder_command_config", "1", "midship_command"));
}

void SailingLogicNewNode::setupSailCommand() 
{
	m_sailCommand.setCommandValues( m_db.retrieveCellAsInt("sail_command_config", "1", "close_reach_command"),
	        m_db.retrieveCellAsInt("sail_command_config", "1", "run_command"));
}

bool SailingLogicNewNode::getGoingStarboard()
{
    if(m_tackingDirection == 1) return true;
    else return false;
}

void SailingLogicNewNode::manageDatabase(VesselStateMsg* msg, double trueWindDirection, double rudder, double sail, double heading,
                        double distanceToNextWaypoint, double bearingToNextWaypoint){
  //logging
  bool routeStarted = false;
  m_db.insertDataLog(
    msg,
    rudder,
    sail,
    0,
    0,                                               
    distanceToNextWaypoint,
    bearingToNextWaypoint,
    heading,
    m_tack,
    getGoingStarboard(),
    m_nextWaypointId,
    trueWindDirection,
    routeStarted
  );
}