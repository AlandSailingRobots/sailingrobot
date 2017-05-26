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
#include "Messages/ExternalControlMsg.h"
#include "Messages/CourseDataMsg.h"
#include "Math/Utility.h"
#include "SystemServices/SysClock.h"
#include <math.h>
#include <algorithm>
#include <cmath>

#define DEFAULT_TWD_BUFFERSIZE 200



FILE* file = fopen("./gps.txt", "w");


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

  msgBus.registerNode(*this, MessageType::StateMessage);
  msgBus.registerNode(*this, MessageType::WindState);
  msgBus.registerNode(*this, MessageType::WaypointData);
  msgBus.registerNode(*this, MessageType::ExternalControl);

  m_tackAngle = 0.872665; //50°

  fprintf( file, "%s,%ss,%s\n", "id", "latitude", "longitude" );
  fflush( file );
}

bool LineFollowNode::init()
{
  setupRudderCommand();

  twdBufferMaxSize = m_db.retrieveCellAsInt("buffer_config", "1", "true_wind");
  if(twdBufferMaxSize == 0)
  twdBufferMaxSize = DEFAULT_TWD_BUFFERSIZE;
  m_dbLogger.startWorkerThread();
  return true;
}

void LineFollowNode::processMessage(const Message* msg)
{
  static int i = 0;
  MessageType type = msg->messageType();

  switch(type)
  {
    case MessageType::ExternalControl:
    m_externalControlActive = ((ExternalControlMsg*)msg)->externalControlActive();
    break;

    case MessageType::StateMessage:
    {
      const StateMessage* stateMsg = static_cast<const StateMessage*>(msg);
      m_Heading = stateMsg->heading();
      m_Latitude = stateMsg->latitude();
      m_Longitude = stateMsg->longitude();
      m_Speed = stateMsg->speed();
      m_Course = stateMsg->course();
    }
    break;
    case MessageType::WindState:
    {
      const WindStateMsg* windStateMsg = static_cast<const WindStateMsg*>(msg);
      m_trueWindSpeed = windStateMsg->trueWindSpeed();
      m_trueWindDir = windStateMsg->trueWindDirection();
      m_apparentWindSpeed = windStateMsg->apparentWindSpeed();
      m_apparentWindDir = windStateMsg->apparentWindDirection();

      if(!m_externalControlActive)
      {
        calculateActuatorPos();
      }else{
        fprintf( file, "%d,%f,%f\n", i, m_Latitude, m_Longitude);
      }
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
      setPrevWaypointData(waypMsg);
    }
    break;
    default:
    return;
  }
}

double LineFollowNode::calculateAngleOfDesiredTrajectory()
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
      {   {-sin(Utility::degreeToRadian(m_Latitude)), cos(Utility::degreeToRadian(m_Latitude )), 0},
      {-cos(Utility::degreeToRadian(m_Latitude ))*sin(Utility::degreeToRadian(m_Latitude )),
        -sin(Utility::degreeToRadian(m_Latitude ))*sin(Utility::degreeToRadian(m_Latitude )),
        cos(Utility::degreeToRadian(m_Latitude ))}};

        std::array<double, 3> bMinusA = { nextWPCoord[0]-prevWPCoord[0], nextWPCoord[1]-prevWPCoord[1], nextWPCoord[2]-prevWPCoord[2]};

        // 2x3 * 1x3
        double phi = atan2(M[0][0]*bMinusA[0] + M[0][1]*bMinusA[1] + M[0][2]*bMinusA[2],   M[1][0]*bMinusA[0] + M[1][1]*bMinusA[1] + M[1][2]*bMinusA[2]);

        return phi;
      }

      void LineFollowNode::calculateActuatorPos()
      {
        /* add pi because trueWindDirection is originally origin of wind but algorithm need direction*/
        double trueWindDirection_radian = Utility::degreeToRadian(m_trueWindDir)+M_PI;

        setPrevWaypointToBoatPos();

        //GET DIRECTION--------
        double currentHeading = getHeading(m_Course, m_Heading, m_Speed, false, false);
        double currentHeading_radian = Utility::degreeToRadian(currentHeading);
        double signedDistance = Utility::calculateSignedDistanceToLine(m_nextWaypointLon, m_nextWaypointLat, m_prevWaypointLon,
          m_prevWaypointLat, m_Longitude, m_Latitude);
          int maxTackDistance = 40; //'r'
          double phi = calculateAngleOfDesiredTrajectory();
          double desiredHeading = phi + (2 * (M_PI / 4)/M_PI) * atan(signedDistance/maxTackDistance); //heading to smoothly join the line
          desiredHeading = Utility::limitRadianAngleRange(desiredHeading);

          //---------------------
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

          // CourseDataMsg not used anymore
          double bearingToNextWaypoint = CourseMath::calculateBTW(m_Longitude, m_Latitude, m_nextWaypointLon, m_nextWaypointLat); //calculated for database
          double distanceToNextWaypoint = CourseMath::calculateDTW(m_Longitude, m_Latitude, m_nextWaypointLon, m_nextWaypointLat);

          MessagePtr courseMsg = std::make_unique<CourseDataMsg>(m_trueWindDir, distanceToNextWaypoint, bearingToNextWaypoint);
          m_MsgBus.sendMessage(std::move(courseMsg)); 

          MessagePtr navMsg = std::make_unique<NavigationControlMsg>(desiredHeading, 0, false, m_tack, getGoingStarboard(), NavigationState::sailToWaypoint);
          m_MsgBus.sendMessage(std::move(navMsg));
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
