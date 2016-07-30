/****************************************************************************************
 *
 * File:
 * 		RoutingNode.h
 *
 * Purpose:
 *		Sailing Logic, calculates actuator positions to sail to the right direction
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/

#include "RoutingNode.h"
#include "Messages/WaypointDataMsg.h"
#include "Messages/ActuatorPositionMsg.h"
#include "utility/Utility.h"
#include "utility/SysClock.h"

#define DEFAULT_TWD_BUFFERSIZE 200

RoutingNode::RoutingNode(MessageBus& msgBus, DBHandler& db)
:  Node(NodeID::SailingLogic, msgBus),
    m_nextWaypointLon(0),
    m_nextWaypointLat(0),
    m_nextWaypointDeclination(0),
    m_nextWaypointRadius(0),
    m_db(db), m_dbLogger(10, m_db),
    m_waypointRouting(m_nextWaypointLon, m_nextWaypointLat, m_nextWaypointRadius,
        atof(m_db.retrieveCell("waypoint_routing_config", "1", "radius_ratio").c_str()),
        atof(m_db.retrieveCell("course_calculation_config", "1", "tack_angle").c_str()),
        atof(m_db.retrieveCell("course_calculation_config", "1", "tack_max_angle").c_str()),
        atof(m_db.retrieveCell("course_calculation_config", "1", "tack_min_speed").c_str()),
        atof(m_db.retrieveCell("course_calculation_config", "1", "sector_angle").c_str()),
        atof(m_db.retrieveCell("waypoint_routing_config", "1", "max_command_angle ").c_str()),
        atof(m_db.retrieveCell("waypoint_routing_config", "1", "rudder_speed_min").c_str())
        )
{
    msgBus.registerNode(this, MessageType::VesselState);
    msgBus.registerNode(this, MessageType::WaypointData);
}

bool RoutingNode::init()
{
    setupRudderCommand();
    setupSailCommand();
    twdBufferMaxSize = m_db.retrieveCellAsInt("buffer_config", "1", "true_wind");
	if(twdBufferMaxSize == 0)
		twdBufferMaxSize = DEFAULT_TWD_BUFFERSIZE;
	m_dbLogger.startWorkerThread();
    return true;
}

void RoutingNode::processMessage(const Message* msg)
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
            m_waypointRouting.setWaypointData(m_nextWaypointLon, m_nextWaypointLat, m_nextWaypointRadius);
        }
		break;
	default:
		return;
	}
}

void RoutingNode::calculateActuatorPos(VesselStateMsg* msg)
{
    if(not msg->gpsOnline())
    {
        Logger::error("GPS not online, using values from last iteration");
        return;
    }

    int heading = getHeading(msg->gpsHeading(), msg->compassHeading(), msg->speed(), false, false);
    double trueWindDirection = Utility::getTrueWindDirection(msg->windDir(), msg->windSpeed(), msg->speed(), msg->compassHeading(), twdBuffer, twdBufferMaxSize);

    double rudderCommand, sailCommand;

    m_waypointRouting.getCommands(rudderCommand, sailCommand, msg->longitude(), msg->latitude(), m_nextWaypointRadius,
	  trueWindDirection, heading, msg->gpsHeading(), msg->speed(), msg->compassHeading(), msg->windDir());

    rudderCommand = m_rudderCommand.getCommand(rudderCommand);
	  sailCommand = m_sailCommand.getCommand(sailCommand);

    ActuatorPositionMsg *actuatorMsg = new ActuatorPositionMsg(NodeID::RudderActuator, nodeID(), rudderCommand, sailCommand);
    m_MsgBus.sendMessage(actuatorMsg);
    ActuatorPositionMsg *actuatorMsg2 = new ActuatorPositionMsg(NodeID::SailActuator, nodeID(), rudderCommand, sailCommand);
    m_MsgBus.sendMessage(actuatorMsg2);

    //create timestamp----
    std::string timestamp_str=SysClock::timeStampStr();
    timestamp_str+=".";
    timestamp_str+= std::to_string(SysClock::millis());
    //--------------------

    m_dbLogger.log(msg, rudderCommand, sailCommand, 0, 0, m_waypointRouting.getDTW(), m_waypointRouting.getBTW(), m_waypointRouting.getCTS(), m_waypointRouting.getTack(), m_waypointRouting.getGoingStarboard(), m_nextWaypointId, trueWindDirection, false,timestamp_str);
}

int RoutingNode::getHeading(int gpsHeading, int compassHeading, double gpsSpeed, bool mockPosition,bool getHeadingFromCompass) {
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

int RoutingNode::getMergedHeading(int gpsHeading, int compassHeading, bool increaseCompassWeight){
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

void RoutingNode::setupRudderCommand()
{
	m_rudderCommand.setCommandValues(m_db.retrieveCellAsInt("rudder_command_config", "1","extreme_command"),
	        m_db.retrieveCellAsInt("rudder_command_config", "1", "midship_command"));
}

void RoutingNode::setupSailCommand()
{
	m_sailCommand.setCommandValues( m_db.retrieveCellAsInt("sail_command_config", "1", "close_reach_command"),
	        m_db.retrieveCellAsInt("sail_command_config", "1", "run_command"));
}

/*void RoutingNode::manageDatabase(VesselStateMsg* msg, double trueWindDirection, double rudder, double sail){
  //logging
  bool routeStarted = false;
  m_db.insertDataLog(
    msg,
    rudder,
    sail,
    0,
    0,
    m_waypointRouting.getDTW(),
    m_waypointRouting.getBTW(),
    m_waypointRouting.getCTS(),
    m_waypointRouting.getTack(),
    m_waypointRouting.getGoingStarboard(),
    m_nextWaypointId,
    trueWindDirection,
    routeStarted
  );
}*/
