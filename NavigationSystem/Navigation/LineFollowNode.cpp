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
*    Algorithm inspired and modified from: 
*    - Luc Jaulin and Fabrice Le Bars "An Experimental Validation of a Robust Controller with 
*       the VAIMOS Autonomous Sailboat" [1];
*    - Jon Melin, Kjell Dahl and Matia Waller "Modeling and Control for an Autonomous Sailboat:
*       A Case Study" [2].
*
*    Info about Tacking and Beating : https://en.wikipedia.org/wiki/Tacking_(sailing)
*
***************************************************************************************/

#include "LineFollowNode.h"

const int INITIAL_SLEEP = 2000;  //in milliseconds
const float NO_COMMAND = -2000; 



LineFollowNode::LineFollowNode(MessageBus& msgBus, DBHandler& db): ActiveNode(NodeID::SailingLogic, msgBus), m_db(db),
m_externalControlActive(false), 
m_nextWaypointLon(0), m_nextWaypointLat(0), m_nextWaypointRadius(0),
m_prevWaypointLon(0), m_prevWaypointLat(0), m_prevWaypointRadius(0),
m_TackDirection(1)
{
    msgBus.registerNode(*this, MessageType::ExternalControl);
    msgBus.registerNode(*this, MessageType::StateMessage);
    msgBus.registerNode(*this, MessageType::WindState);
    msgBus.registerNode(*this, MessageType::WaypointData);

    m_LoopTime = 0.5;

    m_IncidenceAngle = Utility::degreeToRadian(45);
    m_MaxDistanceFromLine = 40;

    m_CloseHauledAngle = Utility::degreeToRadian(45);
    m_TackingDistance = 20;
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

    if(waypMsg->prevId() == 0)
    {   //Set previous waypoint to boat position
        m_prevWaypointLon = m_VesselLon;
        m_prevWaypointLat = m_VesselLat;
        m_prevWaypointRadius = 15;
    }
    else
    {   //Set previous waypoint to previously harvested waypoint
        m_prevWaypointLon = waypMsg->prevLongitude();
        m_prevWaypointLat = waypMsg->prevLatitude();
        m_prevWaypointRadius = waypMsg->prevRadius();
    }
}

double LineFollowNode::calculateAngleOfDesiredTrajectory()
{
    const int earthRadius = 6371000; //meters

    std::array<double, 3> prevWPCoord = {   
        earthRadius * cos(Utility::degreeToRadian(m_prevWaypointLat)) * cos(Utility::degreeToRadian(m_prevWaypointLon)),
        earthRadius * cos(Utility::degreeToRadian(m_prevWaypointLat)) * sin(Utility::degreeToRadian(m_prevWaypointLon)),
        earthRadius * sin(Utility::degreeToRadian(m_prevWaypointLat))};

    std::array<double, 3> nextWPCoord = {  
        earthRadius * cos(Utility::degreeToRadian(m_nextWaypointLat)) * cos(Utility::degreeToRadian(m_nextWaypointLon)),
        earthRadius * cos(Utility::degreeToRadian(m_nextWaypointLat)) * sin(Utility::degreeToRadian(m_nextWaypointLon)),
        earthRadius * sin(Utility::degreeToRadian(m_nextWaypointLat))};

    double M[2][3] = {
        {-sin(Utility::degreeToRadian(m_VesselLon)), cos(Utility::degreeToRadian(m_VesselLon )), 0},
        {-cos(Utility::degreeToRadian(m_VesselLon ))*sin(Utility::degreeToRadian(m_VesselLat )),
        -sin(Utility::degreeToRadian(m_VesselLon ))*sin(Utility::degreeToRadian(m_VesselLat )),
        cos(Utility::degreeToRadian(m_VesselLat ))}
        };

    std::array<double, 3> bMinusA = { nextWPCoord[0]-prevWPCoord[0], nextWPCoord[1]-prevWPCoord[1], nextWPCoord[2]-prevWPCoord[2]};

    // 2x3 * 3x1
    double phi = atan2(M[0][0]*bMinusA[0] + M[0][1]*bMinusA[1] + M[0][2]*bMinusA[2],
        M[1][0]*bMinusA[0] + M[1][1]*bMinusA[1] + M[1][2]*bMinusA[2]);

    return phi;  // in north east down reference frame.
}

float LineFollowNode::calculateTargetCourse()
{
    /* add pi because trueWindDirection is originally origin of wind but algorithm need direction*/
    double trueWindDirection_radian = Utility::degreeToRadian(m_trueWindDir)+M_PI;

    //Calculate signed distance to the line. e.
    double signedDistance = Utility::calculateSignedDistanceToLine(m_nextWaypointLon, m_nextWaypointLat, m_prevWaypointLon,
        m_prevWaypointLat, m_VesselLon, m_VesselLat);

    // Beta.
    double phi = calculateAngleOfDesiredTrajectory();
    double desiredHeading = phi + (2 * (M_PI / 4)/M_PI) * atan(signedDistance/m_TackingDistance);
    desiredHeading = Utility::limitRadianAngleRange(desiredHeading);

    //Change tacking direction when reaching max distance
    if(abs(signedDistance) > m_TackingDistance)
    {
        m_TackDirection = -Utility::sgn(signedDistance);
    }

    //Check if tacking is needed-----
    //tacking may or may not be needed.
    if( (cos(trueWindDirection_radian - desiredHeading) + cos(m_CloseHauledAngle) < 0) || (cos(trueWindDirection_radian - phi) + cos(m_CloseHauledAngle) < 0))
    {
        desiredHeading = M_PI + trueWindDirection_radian - m_TackDirection * m_CloseHauledAngle;/* sail around the wind direction */
        desiredHeading = Utility::limitRadianAngleRange(desiredHeading);
    }


    return desiredHeading;
}


 
void LineFollowNode::LineFollowNodeThreadFunc(ActiveNode* nodePtr)
{
    LineFollowNode* node = dynamic_cast<LineFollowNode*> (nodePtr);

    std::this_thread::sleep_for(std::chrono::milliseconds( INITIAL_SLEEP ));

    Timer timer;
    timer.start();

    while(true)
    {
        float targetCourse = node->calculateTargetCourse();
        MessagePtr LocalNavMsg = std::make_unique<LocalNavigationMsg>(targetCourse, NO_COMMAND, node->m_BeatingState, node->m_TargetTackStarboard);
        node->m_MsgBus.sendMessage( std::move( LocalNavMsg ) );

        timer.sleepUntil(node->m_LoopTime);
        timer.reset();
    }
}