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

#pragma once

#include "Node.h"
#include "Messages/VesselStateMsg.h"
#include "Messages/WaypointDataMsg.h"
#include "dbhandler/DBHandler.h"
#include "sailcommand/SailCommand.h"
#include "ruddercommand/RudderCommand.h"
#include "coursecalculation/CourseMath.h"


class SailingLogicNewNode : public Node {
public:
	SailingLogicNewNode(MessageBus& msgBus, DBHandler& db);

	bool init();

	void processMessage(const Message* message);


	float m_gpsHeadingWeight;

private:
	DBHandler m_db;

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
    int     m_waypointCount;
    double  m_maxCommandAngle, m_maxSailAngle, m_minSailAngle;
    double  m_tackAngle;
    int     m_tackingDirection;
	
    CourseMath m_courseMath;
	RudderCommand m_rudderCommand;
	SailCommand m_sailCommand;

	std::vector<float> twdBuffer;
	unsigned int twdBufferMaxSize;

    double calculateSignedDistance(VesselStateMsg* msg);
    double calculateAngleOfDesiredTrajectory(VesselStateMsg* msg);
	void calculateActuatorPos(VesselStateMsg* msg);
    void setPrevWaypointData(WaypointDataMsg* waypMsg, VesselStateMsg* vesselMsg);

	virtual int getHeading(int gpsHeading, int compassHeading, double gpsSpeed, bool mockPosition, bool getHeadingFromCompass);
  //Calculates a smooth transition between the compass and the gps. Do not call directly, use getHeading()
	int getMergedHeading(int gpsHeading, int compassHeading, bool increaseCompassWeight);
	void setupRudderCommand();
	void setupSailCommand();
    bool getGoingStarboard();

	void manageDatabase(VesselStateMsg* msg, double trueWindDirection, double rudder, double sail, double heading,
                        double distanceToNextWaypoint, double bearingToNextWaypoint);
};