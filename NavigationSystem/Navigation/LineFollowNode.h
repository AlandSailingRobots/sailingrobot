/****************************************************************************************
*
* File:
* 		LineFollowNode.h
*
* Purpose:
*		This class computes the actuator positions of the boat in order to follow
*    lines given by the waypoints.
*
* Developer Notes:
*	 Algorithm inspired and modified from Luc Jaulin and
*    Fabrice Le Bars  "An Experimental Validation of a Robust Controller with the VAIMOS
*    Autonomous Sailboat" and "Modeling and Control for an Autonomous Sailboat: A
*    Case Study" from Jon Melin, Kjell Dahl and Matia Waller
*
*	 Info about Tacking and Beating : https://en.wikipedia.org/wiki/Tacking_(sailing)
*
***************************************************************************************/

#pragma once

#include "MessageBus/ActiveNode.h"
#include "Messages/StateMessage.h"
#include "Messages/WindStateMsg.h"
#include "Messages/WaypointDataMsg.h"
#include "DataBase/DBHandler.h"
#include "DataBase/DBLogger.h"
#include "Messages/NavigationControlMsg.h"
#include "waypointrouting/RudderCommand.h"
#include "Math/CourseMath.h"


class LineFollowNode : public ActiveNode {
public:
	LineFollowNode(MessageBus& msgBus, DBHandler& db);
	~LineFollowNode();

	bool init();
	void start();
	void processMessage(const Message* message);

//	float m_gpsHeadingWeight;

private:

	DBHandler &m_db;

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

	bool 	m_externalControlActive;

	bool    m_tack = false;
	double  m_tackAngle;
	int     m_tackingDirection;

	float 	m_Heading;
	double	m_Latitude;
	double	m_Longitude;
	double	m_Speed;
	double  m_Course;

	double m_trueWindSpeed;
	double m_trueWindDir;
	double m_apparentWindSpeed;
	double m_apparentWindDir;

/*
	const double NORM_RUDDER_COMMAND = 0.5166; // getCommand() take a value between -1 and 1 so we need to normalize the command correspond to 29.6 degree
	const double NORM_SAIL_COMMAND = 0.6958;

	RudderCommand m_rudderCommand;

	std::vector<float> twdBuffer;
	unsigned int twdBufferMaxSize;
*/
	static void LineFollowNodeThreadFunc(ActiveNode* nodePtr);

	void processStateMessage(const StateMessage* stateMsg);
	void processWindStateMessage(const WindStateMsg* windStateMsg);
	void processWaypointMessage(WaypointDataMsg* waypMsg);
	void setPrevWaypointData(WaypointDataMsg* waypMsg);

	double calculateAngleOfDesiredTrajectory();
	double calculateDesiredCourse();

//	virtual int getHeading(int gpsHeading, int compassHeading, double gpsSpeed, bool mockPosition, bool getHeadingFromCompass);
	//Calculates a smooth transition between the compass and the gps. Do not call directly, use getHeading()
//	int getMergedHeading(int gpsHeading, int compassHeading, bool increaseCompassWeight);

	bool getGoingStarboard();
	void setPrevWaypointToBoatPos();

};
