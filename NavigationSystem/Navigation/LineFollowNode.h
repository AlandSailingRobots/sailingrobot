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

#include "MessageBus/ActiveNode.h"
#include "Messages/StateMessage.h"
#include "Messages/WindStateMsg.h"
#include "Messages/WaypointDataMsg.h"
#include "DataBase/DBHandler.h"
#include "DataBase/DBLogger.h"
#include "Messages/NavigationControlMsg.h"
#include "waypointrouting/RudderCommand.h"
#include "Math/CourseMath.h"

#include <atomic>


class LineFollowNode : public ActiveNode {
public:
	LineFollowNode(MessageBus& msgBus, DBHandler& db);
	~LineFollowNode();

	bool init();
	void start();
	void stop();
	void processMessage(const Message* message);
	void updateConfigsFromDB();

//	float m_gpsCourseWeight;

private:

	DBHandler &m_db;
	std::mutex m_lock;
	std::atomic<bool> m_Running;

	double 	m_LoopTime;					// units : seconds (ex: 0.5 s)
	double 	m_MaxTackDistance;			// units : meters
	double  m_tackAngle;				// units : radian

	int 	m_nextWaypointId;
	double 	m_nextWaypointLon;			// units : East(+) or West(-)  [0-180]
	double 	m_nextWaypointLat;			// units : North(+) or South(-) [0-90]
	int 	m_nextWaypointDeclination;	// NOTE : units ?
	int 	m_nextWaypointRadius;		// NOTE : units ?
	int 	m_prevWaypointId;
	double 	m_prevWaypointLon;			// units : East(+) or West(-)  [0-180]
	double 	m_prevWaypointLat;			// units : North(+) or South(-) [0-90]
	int 	m_prevWaypointDeclination;	// NOTE : units ?
	int 	m_prevWaypointRadius;		// NOTE : units ?

	bool 	m_externalControlActive;

	bool    m_tack = false;
	int     m_tackingDirection;

	float 	m_Heading;		// units : [0-359] degrees
	double	m_Latitude;		// units : East(+) or West(-)  [0-180]
	double	m_Longitude;	// units : North(+) or South(-) [0-90]
	double	m_Speed;		// units : m/s km/h or knots
	double  m_Course;		// units : [0-359] degrees

	double m_trueWindSpeed;			// units : m/s km/h or knots
	double m_trueWindDir;			// units : [0-359] degrees
	double m_apparentWindSpeed;		// units : m/s km/h or knots
	double m_apparentWindDir;		// units : [0-359] degrees

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
