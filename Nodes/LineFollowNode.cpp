 /****************************************************************************************
 *
 * File:
 * 		LineFollowNode.cpp
 *
 * Purpose:
 *		This class computes the actuator positions of the boat in order to follow
 *    	lines given by the waypoints.
 *
 * Developer Notes: algorithm inspired and modified from Luc Jaulin and
 *    	Fabrice Le Bars  "An Experimental Validation of a Robust Controller with the VAIMOS
 *    	Autonomous Sailboat" and "Modeling and Control for an Autonomous Sailboat: A
 *    	Case Study" from Jon Melin, Kjell Dahl and Matia Waller
 *
 * Configurable Values
 * 		These are values which are configurable and should be changed for different boats
 * 			* NORM_RUDDER_COMMAND		The maximum angle the rudder can turn to in radians
 * 			* NORM_SAIL_COMMAND			The maximum angle of the sail in radians
 *
 *
 ***************************************************************************************/

#include "LineFollowNode.h"
#include "Messages/ActuatorPositionMsg.h"
#include "Messages/ExternalControlMsg.h"
#include "Messages/CourseDataMsg.h"
#include "utility/Utility.h"
#include "utility/SysClock.h"
#include <math.h>
#include <algorithm>
#include <cmath>
#include "WRSC.h"


#define DEFAULT_TWD_BUFFERSIZE 200

// These values correspond to the angle of the sail/Rudder at its maximum position in radians

/*#define MAX_RUDDER_COMMAND 		M_PI / 6 // 29.9846
#define MIN_SAIL_COMMAND 		0.6958 // 42 Degrees
#define MIN_SAIL_COMMAND 		M_PI / 32.0f;
#define TACK_ANGLE				0.872665; //50°*/

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
    m_externalControlActive(false),
    m_tackingDirection(1)
{
    msgBus.registerNode(*this, MessageType::VesselState);
    msgBus.registerNode(*this, MessageType::WaypointData);
    msgBus.registerNode(*this, MessageType::ExternalControl);

    m_maxCommandAngle = MAX_RUDDER_COMMAND;
    m_maxSailAngle = MAX_SAIL_COMMAND;
    m_minSailAngle = MIN_SAIL_COMMAND;
    m_tackAngle = TACK_ANGLE;
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
    case MessageType::ExternalControl:
         m_externalControlActive = ((ExternalControlMsg*)msg)->externalControlActive();
        break;
	case MessageType::VesselState:
        if(not m_externalControlActive)
        {
            calculateActuatorPos((VesselStateMsg*)msg);
        }
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

	if(std::isnan(msg->longitude()) || std::isnan(msg->latitude()))
	{
		Logger::warning("Invalid GPS coords");
		return;
	}

    double trueWindDirection = Utility::getTrueWindDirection(msg->windDir(), msg->windSpeed(),
                                                             msg->speed(), msg->compassHeading(), twdBuffer, twdBufferMaxSize);
    /* add pi because trueWindDirection is originally origin of wind but algorithm need direction*/
    double trueWindDirection_radian = Utility::degreeToRadian(trueWindDirection)+M_PI;

    setPrevWaypointToBoatPos(msg);

    //GET DIRECTION--------
    double currentHeading = getHeading(msg->gpsHeading(), msg->compassHeading(), msg->speed(), false, false);
    double currentHeading_radian = Utility::degreeToRadian(currentHeading);
    double signedDistance = Utility::calculateSignedDistanceToLine(m_nextWaypointLon, m_nextWaypointLat, m_prevWaypointLon,
                                                                   m_prevWaypointLat, msg->longitude(), msg->latitude());
    int maxTackDistance = 10; //'r'
    double phi = calculateAngleOfDesiredTrajectory(msg);
    double desiredHeading = phi + (2 * (M_PI / 4)/M_PI) * atan(signedDistance/maxTackDistance); //heading to smoothly join the line
    desiredHeading = Utility::limitRadianAngleRange(desiredHeading);
    //---------------------

    static bool changingTack = false;
    //Change tacking direction when reaching max distance
    if(abs(signedDistance) > maxTackDistance)
    {
    	if(m_tack)
    	{
			if(not changingTack)
			{
				m_tackingDirection = -Utility::sgn(signedDistance);
				changingTack = true;
				Logger::info("Tack swap");
			}
        }
    }
    else
    {
    	changingTack = false;
    }
    //--------------------------------------------------

    //Logger::info("Distance from line: %d WindSpeed: %f", abs(signedDistance), msg->windSpeed());
    //Check if tacking is needed-----
    if( (cos(trueWindDirection_radian - desiredHeading) + cos(m_tackAngle) < 0) || (cos(trueWindDirection_radian - phi) + cos(m_tackAngle) < 0))
    {
    	float windAngle = Utility::radianToDegree(cos(trueWindDirection_radian - desiredHeading) + cos(m_tackAngle));
    	//Logger::info("Tacking: %f WindAngle %f TWD: %f Heading: %d", cos(trueWindDirection_radian - desiredHeading), windAngle, trueWindDirection, msg->compassHeading());
        if(!m_tack) /* initialize tacking direction */
        {
        	Logger::info("Begin tack");
            m_tackingDirection = -Utility::sgn(currentHeading_radian-(fmod(trueWindDirection_radian+M_PI, 2*M_PI) - M_PI));
            m_tack = true;
        }

        desiredHeading = M_PI + trueWindDirection_radian - m_tackingDirection * m_tackAngle;/* sail around the wind direction */
        desiredHeading = Utility::limitRadianAngleRange(desiredHeading);
    }
    else if(m_tack)
	{
		m_tack = false;
		Logger::info("End tack");
	}
    //-------------------------------

    double rudderCommand, sailCommand;

    //SET RUDDER-------
    if(cos(currentHeading_radian - desiredHeading) < 0) //if boat heading is too far away from desired heading
    {
        rudderCommand = m_maxCommandAngle * Utility::sgn(sin(currentHeading_radian - desiredHeading));
    }
    else
    {
        rudderCommand = m_maxCommandAngle * sin(currentHeading_radian - desiredHeading);
    }
    //-----------------

    //SET SAIL---------

    // QUICKFIX for WRSC
	double boatHeading_rad = msg->compassHeading() * M_PI /180.0; //Simon : i don't trust currentHeading
    double windDirection_raw = Utility::degreeToRadian(msg->windDir()); // degree from north
    double windSpeed_raw = msg->windSpeed(); // degree from north
    double windDirection = Utility::fmod_2PI_pos(windDirection_raw * M_PI /180.0 + boatHeading_rad + M_PI);
	sailCommand = m_maxSailAngle/2.0 * (cos(windDirection * M_PI /180.0 + M_PI - desiredHeading) + 1);

    //sailCommand = m_minSailAngle;
//    double apparentWindDirection = Utility::getApparentWindDirection(msg->windDir(),
//                    msg->windSpeed(), msg->speed(), currentHeading_radian, trueWindDirection_radian)*M_PI/180;
//
//    apparentWindDirection = ( (apparentWindDirection+M_PI>M_PI) ? (apparentWindDirection-M_PI):(apparentWindDirection+M_PI) );
//
//    sailCommand = fabs(((m_minSailAngle - m_maxSailAngle) / M_PI) * fabs(apparentWindDirection) + m_maxSailAngle);/*!!! on some pc abs only ouptut an int (ubuntu 14.04 gcc 4.9.3)*/
//
//    if (cos(apparentWindDirection+M_PI) + cos(m_maxSailAngle) <0 )
//    {
//       sailCommand = m_minSailAngle;
//    }

    //------------------
	// Normalise and get the PWM value we need
    int rudderCommand_norm = m_rudderCommand.getCommand(rudderCommand / m_maxCommandAngle);
    int sailCommand_norm = m_sailCommand.getCommand(sailCommand / m_maxSailAngle);

    //Logger::info("[Sail] cmd: %d, Rudder: %d sc: %f", sailCommand_norm, rudderCommand_norm, sailCommand);


    //Send messages----
    MessagePtr actuatorMsg = std::make_unique<ActuatorPositionMsg>(rudderCommand_norm, sailCommand_norm);
    m_MsgBus.sendMessage(std::move(actuatorMsg));

    //------------------

    double bearingToNextWaypoint = CourseMath::calculateBTW(msg->longitude(), msg->latitude(), m_nextWaypointLon, m_nextWaypointLat); //calculated for database
    double distanceToNextWaypoint = CourseMath::calculateDTW(msg->longitude(), msg->latitude(), m_nextWaypointLon, m_nextWaypointLat);

    //double appWind_degree = Utility::radianToDegree(apparentWindDirection);

    MessagePtr courseMsg = std::make_unique<CourseDataMsg>(msg->windDir(), distanceToNextWaypoint, bearingToNextWaypoint);
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
	return Utility::addDeclinationToHeading(getMergedHeading(gpsHeading, compassHeading, gpsForbidden), m_nextWaypointDeclination);
    // if(mockPosition) { //TODO - MOCK
    //     return position->getHeading(); //OUTCOMMENTED FOR NOW UNTIL WE FIGURE OUT MOCK
    // }

	/*if (getHeadingFromCompass) {
		//Should return compass heading if below one knot and not currently merging and vice versa
    	return Utility::addDeclinationToHeading(getMergedHeading(gpsHeading, compassHeading, gpsForbidden), m_nextWaypointDeclination);
	}
    return gpsHeading;*/
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
	// KEEP THIS USE IT NORMALLY, GET RID OF WRSC
	// MAX and Middle
	/*m_rudderCommand.setCommandValues(m_db.retrieveCellAsInt("rudder_command_config", "1","extreme_command"),
	        m_db.retrieveCellAsInt("rudder_command_config", "1", "midship_command"));*/

	// FOR WRSC, QUICK HACK
#if BOAT_TYPE == BOAT_ENSTA_GRAND
	m_rudderCommand.setCommandValues( RUDDER_MID_US, RUDDER_MID_US);
#else
	// Normally max
	m_rudderCommand.setCommandValues( RUDDER_MID_US, RUDDER_MAX_US);
#endif
}

void LineFollowNode::setupSailCommand()
{
	// KEEP THIS USE IT NORMALLY, GET RID OF WRSC
	// Close and
	/*m_sailCommand.setCommandValues( m_db.retrieveCellAsInt("sail_command_config", "1", "close_reach_command"),
	        m_db.retrieveCellAsInt("sail_command_config", "1", "run_command"));*/

	// FOR WRSC, QUICK HACK
#if BOAT_TYPE == BOAT_ENSTA_GRAND
	m_sailCommand.setCommandValues( SAIL_MAX_US, SAIL_MIN_US);
#else
	// For little boat swap
	m_sailCommand.setCommandValues( SAIL_MAX_US, SAIL_MIN_US);
#endif
}

bool LineFollowNode::getGoingStarboard()
{
    if(m_tackingDirection == 1) return true;
    else return false;
}

void LineFollowNode::setPrevWaypointToBoatPos(VesselStateMsg* msg) //If boat passed waypoint or enters it, set new line from boat to waypoint.
{                                                                  //Used if boat has to stay within waypoint for a set amount of time.
    double distanceAfterWaypoint = Utility::calculateWaypointsOrthogonalLine(m_nextWaypointLon, m_nextWaypointLat, m_prevWaypointLon,
            m_prevWaypointLat, msg->longitude(), msg->latitude());

    double DTW = CourseMath::calculateDTW(msg->longitude(), msg->latitude(), m_nextWaypointLon, m_nextWaypointLat);
    
    if(distanceAfterWaypoint > 0 ||  DTW < m_nextWaypointRadius)
    {
        m_prevWaypointLon = msg->longitude();
        m_prevWaypointLat = msg->latitude();
    }
}
