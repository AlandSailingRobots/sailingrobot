/****************************************************************************************
*
* File:
*       LineFollowNode.cpp
*
* Purpose:
*       This class computes the actuator positions of the boat in order to follow
*    lines given by the waypoints.
*
* Developer Notes:
*    Algorithm inspired and modified from Luc Jaulin and
*    Fabrice Le Bars  "An Experimental Validation of a Robust Controller with the VAIMOS
*    Autonomous Sailboat" and "Modeling and Control for an Autonomous Sailboat: A
*    Case Study" from Jon Melin, Kjell Dahl and Matia Waller
*
*    Info about Tacking and Beating : https://en.wikipedia.org/wiki/Tacking_(sailing)
*
***************************************************************************************/

#include "LineFollowNode.h"

const int INITIAL_SLEEP = 2000;  //in milliseconds
const float NO_COMMAND = -2000; 

// FILE* file = fopen("./gps.txt", "w");


LineFollowNode::LineFollowNode(MessageBus& msgBus, DBHandler& db): ActiveNode(NodeID::SailingLogic, msgBus), m_db(db),
m_externalControlActive(false), 
m_nextWaypointLon(0), m_nextWaypointLat(0), m_nextWaypointRadius(0),
m_prevWaypointLon(0), m_prevWaypointLat(0), m_prevWaypointRadius(0),
m_tackingDirection(1)
{
    msgBus.registerNode(*this, MessageType::ExternalControl);
    msgBus.registerNode(*this, MessageType::StateMessage);
    msgBus.registerNode(*this, MessageType::WindState);
    msgBus.registerNode(*this, MessageType::WaypointData);

    m_tackAngle = 0.872665; // in radian (= 50°)
    m_LoopTime = 500; // in miliseconds

//  fprintf( file, "%s,%ss,%s\n", "id", "latitude", "longitude" );
//  fflush( file );
}

LineFollowNode::~LineFollowNode() {}

bool LineFollowNode::init()
{
  //setupRudderCommand();
/*
  twdBufferMaxSize = m_db.retrieveCellAsInt("buffer_config", "1", "true_wind");
  if(twdBufferMaxSize == 0)
  twdBufferMaxSize = DEFAULT_TWD_BUFFERSIZE;
*/
    return true;
}

void LineFollowNode::start()
{
  runThread(LineFollowNodeThreadFunc);
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
    default:
        return;
    }
}

void LineFollowNode::processStateMessage(const StateMessage* vesselStateMsg )
{
    m_VesselHeading = vesselStateMsg->heading();
    m_VesselLat = vesselStateMsg->latitude();
    m_VesselLon = vesselStateMsg->longitude();
    m_VesselSpeed = vesselStateMsg->speed();
    m_VesselCourse = vesselStateMsg->course();
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
    m_nextWaypointLon = waypMsg->nextLongitude();
    m_nextWaypointLat = waypMsg->nextLatitude();
    m_nextWaypointRadius = waypMsg->nextRadius();

    if(waypMsg->prevId() == 0) //Set previous waypoint to boat position
    {
        m_prevWaypointLon = m_VesselLon;
        m_prevWaypointLat = m_VesselLat;
        m_prevWaypointRadius = 15;
    }
    else //Set previous waypoint to previously harvested waypoint
    {
        m_prevWaypointLon = waypMsg->prevLongitude();
        m_prevWaypointLat = waypMsg->prevLatitude();
        m_prevWaypointRadius = waypMsg->prevRadius();
    }
}


double LineFollowNode::calculateAngleOfDesiredTrajectory()
{
    const int earthRadius = 6371000; //m

    std::array<double, 3> prevWPCoord = {   
        earthRadius * cos(Utility::degreeToRadian(m_prevWaypointLat)) * cos(Utility::degreeToRadian(m_prevWaypointLon)),
        earthRadius * cos(Utility::degreeToRadian(m_prevWaypointLat)) * sin(Utility::degreeToRadian(m_prevWaypointLon)),
        earthRadius * sin(Utility::degreeToRadian(m_prevWaypointLat))};

    std::array<double, 3> nextWPCoord = {  
        earthRadius * cos(Utility::degreeToRadian(m_nextWaypointLat)) * cos(Utility::degreeToRadian(m_nextWaypointLon)),
        earthRadius * cos(Utility::degreeToRadian(m_nextWaypointLat)) * sin(Utility::degreeToRadian(m_nextWaypointLon)),
        earthRadius * sin(Utility::degreeToRadian(m_nextWaypointLat))};

    double M[2][3] = {
        {-sin(Utility::degreeToRadian(m_VesselLat)), cos(Utility::degreeToRadian(m_VesselLat )), 0},
        {-cos(Utility::degreeToRadian(m_VesselLat ))*sin(Utility::degreeToRadian(m_VesselLat )),
        -sin(Utility::degreeToRadian(m_VesselLat ))*sin(Utility::degreeToRadian(m_VesselLat )),
        cos(Utility::degreeToRadian(m_VesselLat ))}
        };

    std::array<double, 3> bMinusA = { nextWPCoord[0]-prevWPCoord[0], nextWPCoord[1]-prevWPCoord[1], nextWPCoord[2]-prevWPCoord[2]};

    // 2x3 * 1x3
    double phi = atan2(M[0][0]*bMinusA[0] + M[0][1]*bMinusA[1] + M[0][2]*bMinusA[2],   M[1][0]*bMinusA[0] + M[1][1]*bMinusA[1] + M[1][2]*bMinusA[2]);

    return phi;
}

double LineFollowNode::calculateDesiredCourse()
{
    int maxTackDistance = 40; //'r'

    /* add pi because trueWindDirection is originally origin of wind but algorithm need direction*/
    double trueWindDirection_radian = Utility::degreeToRadian(m_trueWindDir)+M_PI;

//    double currentHeading = getHeading(m_VesselCourse, m_Heading, m_Speed, false, false);
    double currentHeading_radian = Utility::degreeToRadian(m_VesselCourse);

    setPrevWaypointToBoatPos();


    //Get Course
    double signedDistance = Utility::calculateSignedDistanceToLine(m_nextWaypointLon, m_nextWaypointLat, m_prevWaypointLon,
        m_prevWaypointLat, m_VesselLon, m_VesselLat);
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


void LineFollowNode::setPrevWaypointToBoatPos() //If boat passed waypoint or enters it, set new line from boat to waypoint.
{                                                                  //Used if boat has to stay within waypoint for a set amount of time.
    double distanceAfterWaypoint = Utility::calculateWaypointsOrthogonalLine(m_nextWaypointLon, m_nextWaypointLat, m_prevWaypointLon,
    m_prevWaypointLat, m_VesselLon, m_VesselLat);

    double DTW = CourseMath::calculateDTW(m_VesselLon, m_VesselLat, m_nextWaypointLon, m_nextWaypointLat);

    if(distanceAfterWaypoint > 0 ||  DTW < m_nextWaypointRadius)
    {
        m_prevWaypointLon = m_VesselLon;
        m_prevWaypointLat = m_VesselLat;
    }
}
 
void LineFollowNode::LineFollowNodeThreadFunc(ActiveNode* nodePtr)
{
    LineFollowNode* node = dynamic_cast<LineFollowNode*> (nodePtr);

    std::this_thread::sleep_for(std::chrono::milliseconds( INITIAL_SLEEP ));

    Timer timer;
    timer.start();

    while(true)
    {
        double desiredCourse = node->calculateDesiredCourse();
        MessagePtr navMsg = std::make_unique<LocalNavigationMsg>(desiredCourse, 0, false, m_tackingDirection);
        node->m_MsgBus.sendMessage( std::move( navMsg ) );

        timer.sleepUntil(node->m_LoopTime);
        timer.reset();
    }
}