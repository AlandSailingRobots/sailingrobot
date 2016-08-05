 /****************************************************************************************
 *
 * File:
 * 		LineFollowNode.cpp
 *
 * Purpose:
 *		This class computes the actuator positions of the boat in order to follow
 *    lines given by the waypoints.
 *
 * Developer Notes: algorithm inspired and modified from Luc Jaulin and
 *    Fabrice Le Bars  "An Experimental Validation of a Robust Controller with the VAIMOS
 *    Autonomous Sailboat" and "Modeling and Control for an Autonomous Sailboat: A
 *    Case Study" from Jon Melin, Kjell Dahl and Matia Waller
 *
 *
 ***************************************************************************************/

#include "LineFollowNode.h"
#include "Messages/ActuatorPositionMsg.h"
#include "Messages/CourseDataMsg.h"
#include "utility/Utility.h"
#include "utility/SysClock.h"
#include <math.h>
#include <algorithm>
#include <cmath>

#define DEFAULT_TWD_BUFFERSIZE 200
#define NORM_RUDDER_COMMAND 0.5166 // getCommand() take a value between -1 and 1 so we need to normalize the command correspond to 29.6 degree
#define NORM_SAIL_COMMAND 0.6958

LineFollowNode::LineFollowNode(MessageBus& msgBus, DBHandler& db)
:  Node(NodeID::SailingLogic, msgBus), m_db(db), m_dbLogger(5, db),
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
    m_tackingDirection(1)
{
    msgBus.registerNode(*this, MessageType::VesselState);
    msgBus.registerNode(*this, MessageType::WaypointData);

    m_maxCommandAngle = M_PI / 6;
    m_maxSailAngle = M_PI / 4.2f;
    m_minSailAngle = M_PI / 32.0f;
    m_tackAngle = 0.872665; //50°
}

bool LineFollowNode::init()
{
    setupRudderCommand();
    setupSailCommand();
    twdBufferMaxSize = m_db.retrieveCellAsInt("buffer_config", "1", "true_wind");
	if(twdBufferMaxSize == 0)
		twdBufferMaxSize = DEFAULT_TWD_BUFFERSIZE;
	m_dbLogger.startWorkerThread();
    return true;
}

void LineFollowNode::processMessage(const Message* msg)
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

double LineFollowNode::calculateAngleOfDesiredTrajectory(VesselStateMsg* msg)
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

void LineFollowNode::calculateActuatorPos(VesselStateMsg* msg)
{
    if(not msg->gpsOnline())
    {
        Logger::error("GPS not online, using values from last iteration");
        return;
    }

    double trueWindDirection = Utility::getTrueWindDirection(msg->windDir(), msg->windSpeed(),
                                                             msg->speed(), msg->compassHeading(), twdBuffer, twdBufferMaxSize);
    /* add pi because trueWindDirection is originally origin of wind but algorithm need direction*/
    double trueWindDirection_radian = Utility::degreeToRadian(trueWindDirection)+M_PI;

    //GET DIRECTION--------
    double currentHeading = getHeading(msg->gpsHeading(), msg->compassHeading(), msg->speed(), false, false);
    double currentHeading_radian = Utility::degreeToRadian(currentHeading);
    double signedDistance = Utility::calculateSignedDistanceToLine(m_nextWaypointLon, m_nextWaypointLat, m_prevWaypointLon,
                                                                   m_prevWaypointLat, msg->longitude(), msg->latitude());
    int maxTackDistance = 20; //'r'
    double phi = calculateAngleOfDesiredTrajectory(msg);
    double desiredHeading = phi + (2 * (M_PI / 4)/M_PI) * atan(signedDistance/maxTackDistance); //heading to smoothly join the line
    desiredHeading = Utility::limitRadianAngleRange(desiredHeading);
    //---------------------

    //Change tacking direction when reaching max distance
    if(abs(signedDistance) > maxTackDistance)
    {
        m_tackingDirection = -Utility::sgn(signedDistance);
    }
    //--------------------------------------------------

    //Check if tacking is needed-----
    if( (cos(trueWindDirection_radian - desiredHeading) + cos(m_tackAngle) < 0) || (cos(trueWindDirection_radian - phi) + cos(m_tackAngle) < 0))
    {
        if(!m_tack) /* initialize tacking direction */
        {
            m_tackingDirection = -Utility::sgn(currentHeading_radian-(fmod(trueWindDirection_radian+M_PI, 2*M_PI) - M_PI));
            m_tack = true;
        }

        desiredHeading = M_PI + trueWindDirection_radian - m_tackingDirection * m_tackAngle;/* sail around the wind direction */
        desiredHeading = Utility::limitRadianAngleRange(desiredHeading);
    }
    else
    {
        m_tack = false;
    }
    //-------------------------------

    double rudderCommand, sailCommand;

    //SET RUDDER-------
    if(cos(currentHeading_radian - desiredHeading) < 0) //if boat heading is too far away from desired heading
    {
        rudderCommand = -Utility::sgn(msg->speed()) * m_maxCommandAngle * Utility::sgn(sin(currentHeading_radian - desiredHeading));
    }
    else
    {
        rudderCommand = -Utility::sgn(msg->speed()) * m_maxCommandAngle * sin(currentHeading_radian - desiredHeading);
    }
    //-----------------

    //SET SAIL---------
    double apparentWindDirection = Utility::getApparentWindDirection(msg->windDir(),
                    msg->windSpeed(), msg->speed(), currentHeading_radian, trueWindDirection_radian)*M_PI/180;

    apparentWindDirection = ( (apparentWindDirection+M_PI>M_PI) ? (apparentWindDirection-M_PI):(apparentWindDirection+M_PI) );

    sailCommand = fabs(((m_minSailAngle - m_maxSailAngle) / M_PI) * fabs(apparentWindDirection) + m_maxSailAngle);/*!!! on some pc abs only ouptut an int (ubuntu 14.04 gcc 4.9.3)*/

    if (cos(apparentWindDirection+M_PI) + cos(m_maxSailAngle) <0 )
    {
       sailCommand = m_minSailAngle;
    }

    //------------------
    int rudderCommand_norm = m_rudderCommand.getCommand(rudderCommand/NORM_RUDDER_COMMAND);
    int sailCommand_norm = m_sailCommand.getCommand(sailCommand/NORM_SAIL_COMMAND);

    //Send messages----
    MessagePtr actuatorMsg = std::make_unique<ActuatorPositionMsg>(rudderCommand_norm, sailCommand_norm);
    m_MsgBus.sendMessage(std::move(actuatorMsg));

    //------------------

    double bearingToNextWaypoint = m_courseMath.calculateBTW(msg->longitude(), msg->latitude(), m_nextWaypointLon, m_nextWaypointLat); //calculated for database
    double distanceToNextWaypoint = m_courseMath.calculateDTW(msg->longitude(), msg->latitude(), m_nextWaypointLon, m_nextWaypointLat);

    MessagePtr courseMsg = std::make_unique<CourseDataMsg>(trueWindDirection, distanceToNextWaypoint, bearingToNextWaypoint);
    m_MsgBus.sendMessage(std::move(courseMsg));

    //create timestamp----
    std::string timestamp_str=SysClock::timeStampStr();
    timestamp_str+=".";
    timestamp_str+= std::to_string(SysClock::millis());
    //--------------------

    m_dbLogger.log(msg, rudderCommand_norm, sailCommand_norm, 0, 0, distanceToNextWaypoint, bearingToNextWaypoint, desiredHeading, m_tack, getGoingStarboard(), m_nextWaypointId, trueWindDirection, false,timestamp_str);
}

void LineFollowNode::setPrevWaypointData(WaypointDataMsg* waypMsg, VesselStateMsg* vesselMsg)
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

int LineFollowNode::getHeading(int gpsHeading, int compassHeading, double gpsSpeed, bool mockPosition,bool getHeadingFromCompass)
{
	//Use GPS for heading only if speed is higher than 1 m/s
	int useGpsForHeadingMeterSecSpeed = 1;
	bool gpsForbidden = Utility::directionAdjustedSpeed(gpsHeading, compassHeading, gpsSpeed) < useGpsForHeadingMeterSecSpeed;

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

int LineFollowNode::getMergedHeading(int gpsHeading, int compassHeading, bool increaseCompassWeight)
{
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

void LineFollowNode::setupRudderCommand()
{
	m_rudderCommand.setCommandValues(m_db.retrieveCellAsInt("rudder_command_config", "1","extreme_command"),
	        m_db.retrieveCellAsInt("rudder_command_config", "1", "midship_command"));
}

void LineFollowNode::setupSailCommand()
{
	m_sailCommand.setCommandValues( m_db.retrieveCellAsInt("sail_command_config", "1", "close_reach_command"),
	        m_db.retrieveCellAsInt("sail_command_config", "1", "run_command"));
}

bool LineFollowNode::getGoingStarboard()
{
    if(m_tackingDirection == 1) return true;
    else return false;
}

/*
void LineFollowNode::manageDatabase(VesselStateMsg* msg, double trueWindDirection, double rudder, double sail, double heading,
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
}*/
