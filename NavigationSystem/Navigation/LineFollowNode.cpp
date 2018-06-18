/****************************************************************************************
*
* File:
*       LineFollowNode.cpp
*
* Purpose:
*       This class computes the actuator positions of the boat in order to follow the
*    line given by the waypoints.
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

#define DATA_OUT_OF_RANGE -2000
const int INITIAL_SLEEP = 2000;  // milliseconds
const float NO_COMMAND = -1000;


LineFollowNode::LineFollowNode(MessageBus& msgBus, DBHandler& dbhandler): ActiveNode(NodeID::SailingLogic, msgBus),
m_db(dbhandler), m_LoopTime(0.5), m_externalControlActive(false),
m_VesselLat(DATA_OUT_OF_RANGE), m_VesselLon(DATA_OUT_OF_RANGE), 
m_trueWindSpeed(DATA_OUT_OF_RANGE), m_trueWindDir(DATA_OUT_OF_RANGE),
m_nextWaypointLon(DATA_OUT_OF_RANGE), m_nextWaypointLat(DATA_OUT_OF_RANGE), m_nextWaypointRadius(DATA_OUT_OF_RANGE),
m_prevWaypointLon(DATA_OUT_OF_RANGE), m_prevWaypointLat(DATA_OUT_OF_RANGE), m_prevWaypointRadius(DATA_OUT_OF_RANGE),
m_TackDirection(1), m_BeatingMode(false)
{
    msgBus.registerNode(*this, MessageType::ExternalControl);
    msgBus.registerNode(*this, MessageType::StateMessage);
    msgBus.registerNode(*this, MessageType::WindState);
    msgBus.registerNode(*this, MessageType::WaypointData);
    msgBus.registerNode(*this, MessageType::ServerConfigsReceived);

    m_IncidenceAngle = Utility::degreeToRadian(90);
    m_MaxDistanceFromLine = 20;

    m_CloseHauledAngle = Utility::degreeToRadian(45);
    m_BroadReachAngle = Utility::degreeToRadian(30);
    m_TackingDistance = 15;
}

LineFollowNode::~LineFollowNode() {}

bool LineFollowNode::init()
{
    updateConfigsFromDB();
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
    m_LoopTime = m_db.retrieveCellAsDouble("config_line_follow","1","loop_time");
    m_CloseHauledAngle = Utility::degreeToRadian(m_db.retrieveCellAsDouble("config_line_follow","1","close_hauled_angle"));
    m_BroadReachAngle = Utility::degreeToRadian(m_db.retrieveCellAsDouble("config_line_follow","1","broad_reach_angle"));
    m_TackingDistance = m_db.retrieveCellAsDouble("config_line_follow","1","tacking_distance");
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

void LineFollowNode::processStateMessage(const StateMessage* vesselStateMsg )
{
    std::lock_guard<std::mutex> lock_guard(m_lock);

    m_VesselLat = vesselStateMsg->latitude();
    m_VesselLon = vesselStateMsg->longitude();
}

void LineFollowNode::processWindStateMessage(const WindStateMsg* windStateMsg )
{
    std::lock_guard<std::mutex> lock_guard(m_lock);

    m_trueWindSpeed = windStateMsg->trueWindSpeed();
    m_trueWindDir = windStateMsg->trueWindDirection();

    unsigned int twdBufferMaxSize = 200;
    Utility::addValueToBuffer(m_trueWindDir, m_TwdBuffer, twdBufferMaxSize);
}

void LineFollowNode::processWaypointMessage(WaypointDataMsg* waypMsg )
{
    std::lock_guard<std::mutex> lock_guard(m_lock);

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

double LineFollowNode::calculateTargetCourse()
{
    // In the articles the reference frame is East-North-Up. Here the reference frame is North-East-Down.

    std::lock_guard<std::mutex> lock_guard(m_lock);

    if ((m_VesselLat == DATA_OUT_OF_RANGE) || (m_VesselLon == DATA_OUT_OF_RANGE) || 
        (m_trueWindSpeed == DATA_OUT_OF_RANGE) || (m_trueWindDir == DATA_OUT_OF_RANGE) ||
        (m_nextWaypointLon == DATA_OUT_OF_RANGE) || (m_nextWaypointLat == DATA_OUT_OF_RANGE) || (m_nextWaypointRadius == DATA_OUT_OF_RANGE) )
    {
        return DATA_OUT_OF_RANGE;
    }
    else
    {
        // Calculate the angle of the true wind vector.     [1]:(psi)       [2]:(psi_tw).
        double meanTrueWindDir = Utility::meanOfAngles(m_TwdBuffer);
        double trueWindAngle = Utility::limitRadianAngleRange(Utility::degreeToRadian(meanTrueWindDir)+M_PI);
        //float trueWindAngle = Utility::degreeToRadian(m_trueWindDir);

        // Calculate signed distance to the line.           [1] and [2]: (e).
        double signedDistance = Utility::calculateSignedDistanceToLine(m_nextWaypointLon, m_nextWaypointLat, m_prevWaypointLon,
            m_prevWaypointLat, m_VesselLon, m_VesselLat);

        // Calculate the angle of the line to be followed.  [1]:(phi)       [2]:(beta)
        double phi = calculateAngleOfDesiredTrajectory();

        // Calculate the target course in nominal mode.     [1]:(theta_*)   [2]:(theta_r)
        double targetCourse = phi + (2 * m_IncidenceAngle/M_PI) * atan(signedDistance/m_MaxDistanceFromLine);
        targetCourse = Utility::limitRadianAngleRange(targetCourse); // in north east down reference frame.

        // Change tack direction when reaching tacking distance
        if(abs(signedDistance) > m_TackingDistance)
        {
            m_TackDirection = Utility::sgn(signedDistance);
        }

        // Check if the targetcourse is inconsistent with the wind.
        if( (cos(trueWindAngle - targetCourse) + cos(m_CloseHauledAngle) < 0) ||
            ((cos(trueWindAngle - phi) + cos(m_CloseHauledAngle) < 0) and (abs(signedDistance) < m_MaxDistanceFromLine)) )
        {
            // Close hauled mode (Upwind beating mode).
            m_BeatingMode = true;
            targetCourse = M_PI + trueWindAngle + m_TackDirection*m_CloseHauledAngle;
        }
        else if( (cos(trueWindAngle - targetCourse) - cos(m_BroadReachAngle) > 0) ||
                 ((cos(trueWindAngle - phi) - cos(m_BroadReachAngle) > 0) and (abs(signedDistance) < m_MaxDistanceFromLine)) )
        {
            // Broad reach mode (Downwind beating mode).
            m_BeatingMode = true;
            targetCourse = trueWindAngle + m_TackDirection*m_BroadReachAngle;
        }
        else
        {
            m_BeatingMode = false;
        }

        targetCourse = Utility::limitRadianAngleRange(targetCourse);
        targetCourse = Utility::radianToDegree(targetCourse);

        return targetCourse; // in north east down reference frame.
    }
}

bool LineFollowNode::getTargetTackStarboard(double targetCourse)
{
    std::lock_guard<std::mutex> lock_guard(m_lock);

    double meanTrueWindDir = Utility::meanOfAngles(m_TwdBuffer);
    if (sin(Utility::degreeToRadian(targetCourse - meanTrueWindDir)) < 0){
        return true;
    } else {
        return false;
    }
}

void LineFollowNode::ifBoatPassedOrEnteredWP_setPrevWPToBoatPos()
{
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


    while(node->m_Running.load() == true)
    {
        node->ifBoatPassedOrEnteredWP_setPrevWPToBoatPos();
        double targetCourse = node->calculateTargetCourse();
        if (targetCourse != DATA_OUT_OF_RANGE)
        {
            bool targetTackStarboard = node->getTargetTackStarboard(targetCourse);
            MessagePtr LocalNavMsg = std::make_unique<LocalNavigationMsg>((float) targetCourse, NO_COMMAND, node->m_BeatingMode, targetTackStarboard);
            node->m_MsgBus.sendMessage( std::move( LocalNavMsg ) );
        }
        timer.sleepUntil(node->m_LoopTime);
        timer.reset();
    }
}
