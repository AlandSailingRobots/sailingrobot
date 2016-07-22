/****************************************************************************************
 *
 * File:
 * 		SailigLogicOld.h
 *
 * Purpose:
 *		Sailing Logic, makes boat sail to the right place
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
#include "sailcommand/SailCommand.h"
#include "ruddercommand/RudderCommand.h"


class SailingLogicOldNode : public Node {
public:
	SailingLogicOldNode(MessageBus& msgBus, DBHandler& db);

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
	
	DBHandler m_db;
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

	void manageDatabase(double trueWindDirection, SystemStateModel &systemStateModel);
};