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

#pragma once

#include "Node.h"
#include "Messages/VesselStateMsg.h"
#include "waypointrouting/WaypointRouting.h"
#include "dbhandler/DBHandler.h"
#include "dbhandler/DBLogger.h"
#include "sailcommand/SailCommand.h"
#include "ruddercommand/RudderCommand.h"


class RoutingNode : public Node {
public:
	RoutingNode(MessageBus& msgBus, DBHandler& db);

	bool init();

	void processMessage(const Message* message);

	void calculateActuatorPos(VesselStateMsg* msg);

	float m_gpsHeadingWeight;
private:

	int 	m_nextWaypointId;
	double 	m_nextWaypointLon;
	double 	m_nextWaypointLat;
	int 	m_nextWaypointDeclination;
	int 	m_nextWaypointRadius;
	int		m_nextWaypointStayTime;

	DBHandler &m_db;
	DBLogger m_dbLogger;
	WaypointRouting m_waypointRouting;
	RudderCommand m_rudderCommand;
	SailCommand m_sailCommand;

	std::vector<float> twdBuffer;
	unsigned int twdBufferMaxSize;



	virtual int getHeading(int gpsHeading, int compassHeading, double gpsSpeed, bool mockPosition, bool getHeadingFromCompass);
  //Calculates a smooth transition between the compass and the gps. Do not call directly, use getHeading()
	int getMergedHeading(int gpsHeading, int compassHeading, bool increaseCompassWeight);
	void setupRudderCommand();
	void setupSailCommand();

	//void manageDatabase(VesselStateMsg* msg, double trueWindDirection, double rudder, double sail);
};
