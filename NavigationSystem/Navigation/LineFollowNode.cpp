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
#include "Messages/DesiredCourseMsg.h"
#include "Messages/ExternalControlMsg.h"
#include "Math/Utility.h"
#include "SystemServices/SysClock.h"
#include <math.h>
#include <algorithm>
#include <cmath>

#include "SystemServices/Timer.h"
#include <chrono>

#define INITIAL_SLEEP     2000

// FILE* file = fopen("./gps.txt", "w");


LineFollowNode::LineFollowNode(MessageBus& msgBus, DBHandler& db): ActiveNode(NodeID::SailingLogic, msgBus), m_db(db),
m_LoopTime(0.5), m_MaxTackDistance(0), m_tackAngle(0.872665),
m_nextWaypointId(0), m_nextWaypointLon(0), m_nextWaypointLat(0), m_nextWaypointDeclination(0), m_nextWaypointRadius(0),
m_prevWaypointId(0), m_prevWaypointLon(0), m_prevWaypointLat(0), m_prevWaypointDeclination(0), m_prevWaypointRadius(0),
m_externalControlActive(false), m_tackingDirection(1)
{
    msgBus.registerNode(*this, MessageType::StateMessage);
    msgBus.registerNode(*this, MessageType::WindState);
    msgBus.registerNode(*this, MessageType::WaypointData);
    msgBus.registerNode(*this, MessageType::ExternalControl);
    msgBus.registerNode(*this, MessageType::ServerConfigsReceived);

// NOTE : Marc : conversion radian to degree
    //m_tackAngle = 0.872665; // in radian (= 50°)

//  fprintf( file, "%s,%ss,%s\n", "id", "latitude", "longitude" );
//  fflush( file );
}

LineFollowNode::~LineFollowNode() {}

bool LineFollowNode::init()
{
  //setupRudderCommand();
/*
  twdBufferMaxSize = m_db.retrieveCellAsInt("config_buffer", "1", "true_wind");
  if(twdBufferMaxSize == 0)
  twdBufferMaxSize = DEFAULT_TWD_BUFFERSIZE;
*/
    return true;
}

void LineFollowNode::start()
{
    m_Running.store(true);
    runThread(LineFollowNodeThreadFunc);
}

void LineFollowNode::stop()
{
    m_Running.store(false);
    stopThread(this);
}

void LineFollowNode::updateConfigsFromDB()
{
    m_LoopTime = m_db.retrieveCellAsInt("config_LineFollowNode","1","loop_time");
    m_MaxTackDistance = m_db.retrieveCellAsInt("config_LineFollowNode","1","max_tack_distance");
    m_tackAngle = m_db.retrieveCellAsInt("config_LineFollowNode","1","tack_angle");
}

void LineFollowNode::processMessage(const Message* msg)
{
  MessageType type = msg->messageType();
    switch(type)
    {
    case MessageType::ExternalControl:
        m_externalControlActive = ((ExternalControlMsg*)msg)->externalControlActive();
        break;
    case MessageType::StateMessage:
        processStateMessage(static_cast<const StateMessage*>(msg));
        break;
    case MessageType::WindState:
        processWindStateMessage(static_cast<const WindStateMsg*>(msg));
        break;
    case MessageType::WaypointData:
        processWaypointMessage((WaypointDataMsg*)msg);
        break;
    case MessageType::ServerConfigsReceived:
        updateConfigsFromDB();
        break;
    default:
        return;
    }
}

void LineFollowNode::processStateMessage(const StateMessage* stateMsg )
{
    m_Heading = stateMsg->heading();
    m_Latitude = stateMsg->latitude();
    m_Longitude = stateMsg->longitude();
    m_Speed = stateMsg->speed();
    m_Course = stateMsg->course();
}

void LineFollowNode::processWindStateMessage(const WindStateMsg* windStateMsg )
{
    m_trueWindSpeed = windStateMsg->trueWindSpeed();
    m_trueWindDir = windStateMsg->trueWindDirection();
    m_apparentWindSpeed = windStateMsg->apparentWindSpeed();
    m_apparentWindDir = windStateMsg->apparentWindDirection();
}

void LineFollowNode::processWaypointMessage(WaypointDataMsg* waypMsg )
{
    m_nextWaypointId = waypMsg->nextId();
    m_nextWaypointLon = waypMsg->nextLongitude();
    m_nextWaypointLat = waypMsg->nextLatitude();
    m_nextWaypointDeclination = waypMsg->nextDeclination();
    m_nextWaypointRadius = waypMsg->nextRadius();
    setPrevWaypointData(waypMsg);
}

void LineFollowNode::setPrevWaypointData(WaypointDataMsg* waypMsg)
{
    if(waypMsg->prevId() == 0) //Set previous waypoint to boat position
    {
        m_prevWaypointId = 0;
        m_prevWaypointLon = m_Longitude;
        m_prevWaypointLat = m_Latitude;
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



double LineFollowNode::calculateAngleOfDesiredTrajectory()
{
    int earthRadius = 6371000;

    std::array<double, 3> prevWPCoord = {
        earthRadius * cos(Utility::degreeToRadian(m_prevWaypointLat)) * cos(Utility::degreeToRadian(m_prevWaypointLon)),
        earthRadius * cos(Utility::degreeToRadian(m_prevWaypointLat)) * sin(Utility::degreeToRadian(m_prevWaypointLon)),
        earthRadius * sin(Utility::degreeToRadian(m_prevWaypointLat))};

    std::array<double, 3> nextWPCoord = {
        earthRadius * cos(Utility::degreeToRadian(m_nextWaypointLat)) * cos(Utility::degreeToRadian(m_nextWaypointLon)),
        earthRadius * cos(Utility::degreeToRadian(m_nextWaypointLat)) * sin(Utility::degreeToRadian(m_nextWaypointLon)),
        earthRadius * sin(Utility::degreeToRadian(m_nextWaypointLat))};

    double M[2][3] = {
        {-sin(Utility::degreeToRadian(m_Latitude)), cos(Utility::degreeToRadian(m_Latitude )), 0},
        {-cos(Utility::degreeToRadian(m_Latitude ))*sin(Utility::degreeToRadian(m_Latitude )),
        -sin(Utility::degreeToRadian(m_Latitude ))*sin(Utility::degreeToRadian(m_Latitude )),
        cos(Utility::degreeToRadian(m_Latitude ))}
        };

    std::array<double, 3> bMinusA = { nextWPCoord[0]-prevWPCoord[0], nextWPCoord[1]-prevWPCoord[1], nextWPCoord[2]-prevWPCoord[2]};

    // 2x3 * 1x3
    double phi = atan2(M[0][0]*bMinusA[0] + M[0][1]*bMinusA[1] + M[0][2]*bMinusA[2],   M[1][0]*bMinusA[0] + M[1][1]*bMinusA[1] + M[1][2]*bMinusA[2]);

    return phi;
}

double LineFollowNode::calculateDesiredCourse()
{
    int maxTackDistance = 40;

    /* add pi because trueWindDirection is originally origin of wind but algorithm need direction*/
    double trueWindDirection_radian = Utility::degreeToRadian(m_trueWindDir)+M_PI;

//    double currentHeading = getHeading(m_Course, m_Heading, m_Speed, false, false);
    double currentHeading_radian = Utility::degreeToRadian(m_Course);

    setPrevWaypointToBoatPos();


    //Get Course
    double signedDistance = Utility::calculateSignedDistanceToLine(m_nextWaypointLon, m_nextWaypointLat, m_prevWaypointLon,
        m_prevWaypointLat, m_Longitude, m_Latitude);
    double phi = calculateAngleOfDesiredTrajectory();
    double desiredHeading = phi + (2 * (M_PI / 4)/M_PI) * atan(signedDistance/maxTackDistance); //heading to smoothly join the line
    desiredHeading = Utility::limitRadianAngleRange(desiredHeading);

    //Change tacking direction when reaching max distance
    if(abs(signedDistance) > maxTackDistance)
    {
        m_tackingDirection = -Utility::sgn(signedDistance);
    }

    //Check if tacking is needed-----
    //tacking may or may not be needed. Decide if this code is necessary
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

    return desiredHeading;
}

/*


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
            m_gpsCourseWeight = m_gpsCourseWeight - tickRate; //Decrease gps weight
            if (m_gpsCourseWeight < 0.0) m_gpsCourseWeight = 0;
          }else{
            m_gpsCourseWeight = m_gpsCourseWeight + tickRate;
            if (m_gpsCourseWeight > 1.0) m_gpsCourseWeight = 1.0;
          }

          //Difference calculation
          float diff = ((headingGps - headingCompass) + 180 + 360);
          while (diff > 360) diff -= 360;
          diff -= 180;

          //Merge angle calculation
          int returnValue = 360 + headingCompass + (diff * m_gpsCourseWeight);
          while (returnValue > 360) returnValue -= 360;

          return returnValue;
        }

*/


/*
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
*/

bool LineFollowNode::getGoingStarboard()
{
    if(m_tackingDirection == 1) return true;
    else return false;
}

void LineFollowNode::setPrevWaypointToBoatPos() //If boat passed waypoint or enters it, set new line from boat to waypoint.
{                                                                  //Used if boat has to stay within waypoint for a set amount of time.
    double distanceAfterWaypoint = Utility::calculateWaypointsOrthogonalLine(m_nextWaypointLon, m_nextWaypointLat, m_prevWaypointLon,
    m_prevWaypointLat, m_Longitude, m_Latitude);

    double DTW = CourseMath::calculateDTW(m_Longitude, m_Latitude, m_nextWaypointLon, m_nextWaypointLat);

    if(distanceAfterWaypoint > 0 ||  DTW < m_nextWaypointRadius)
    {
        m_prevWaypointLon = m_Longitude;
        m_prevWaypointLat = m_Latitude;
    }
}

void LineFollowNode::LineFollowNodeThreadFunc(ActiveNode* nodePtr)
{
    LineFollowNode* node = dynamic_cast<LineFollowNode*> (nodePtr);

    std::this_thread::sleep_for(std::chrono::milliseconds( INITIAL_SLEEP ));

    Timer timer;
    timer.start();


    while(node->m_Running.load() == true)
    {
        std::lock_guard<std::mutex> lock_guard(node->m_lock);        double desiredCourse = node->calculateDesiredCourse();
        bool goingStarboard = node->getGoingStarboard();
        MessagePtr navMsg = std::make_unique<NavigationControlMsg>(desiredCourse, 0, false, node->m_tack, goingStarboard, NavigationState::sailToWaypoint);
        node->m_MsgBus.sendMessage( std::move( navMsg ) );

        timer.sleepUntil(node->m_LoopTime);
        timer.reset();
    }
}
