 /****************************************************************************************
 *
 * File:
 * 		LineFollowNode.h
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

#pragma once

#include "Node.h"
#include "Messages/VesselStateMsg.h"
#include "Messages/WaypointDataMsg.h"
#include "dbhandler/DBHandler.h"
#include "dbhandler/DBLogger.h"
#include "waypointrouting/SailCommand.h"
#include "waypointrouting/RudderCommand.h"
#include "utility/CourseMath.h"

#include "utility/Timer.h"

class LineFollowNode : public Node {
public:
	LineFollowNode(MessageBus& msgBus, DBHandler& db);

	bool init();

	void processMessage(const Message* message);


	float m_gpsHeadingWeight;

private:
	DBHandler &m_db;
	DBLogger m_dbLogger;

	int 	m_nextWaypointId;
	double 	m_nextWaypointLon;
	double 	m_nextWaypointLat;
	int 	m_nextWaypointDeclination;
	int 	m_nextWaypointRadius;

    int 	m_prevWaypointId;
	double 	m_prevWaypointLon;
	double 	m_prevWaypointLat;
	int 	m_prevWaypointDeclination;
	int 	m_prevWaypointRadius;

    bool    m_tack;
    double  m_maxCommandAngle, m_maxSailAngle, m_minSailAngle;
    double  m_tackAngle;
    int     m_tackingDirection;

	RudderCommand m_rudderCommand;
	SailCommand m_sailCommand;

	std::vector<float> twdBuffer;
	unsigned int twdBufferMaxSize;

    double calculateAngleOfDesiredTrajectory(VesselStateMsg* msg);
	void calculateActuatorPos(VesselStateMsg* msg);
    void setPrevWaypointData(WaypointDataMsg* waypMsg, VesselStateMsg* vesselMsg);

	virtual int getHeading(int gpsHeading, int compassHeading, double gpsSpeed, bool mockPosition, bool getHeadingFromCompass);
  //Calculates a smooth transition between the compass and the gps. Do not call directly, use getHeading()
	int getMergedHeading(int gpsHeading, int compassHeading, bool increaseCompassWeight);
	void setupRudderCommand();
	void setupSailCommand();
    bool getGoingStarboard();
	void setPrevWaypointToBoatPos(VesselStateMsg* msg);

	/*void manageDatabase(VesselStateMsg* msg, double trueWindDirection, double rudder, double sail, double heading,
                        double distanceToNextWaypoint, double bearingToNextWaypoint);*/

	float m_old_diff_v;
	float m_int_diff_v;
	Timer m_timer;
};