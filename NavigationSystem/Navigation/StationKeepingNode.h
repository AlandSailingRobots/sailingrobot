/****************************************************************************************
*
* File:
* 		StationKeepingNode.h
*
* Purpose:
*		This class computes the actuator positions of the boat in order to stay near a waypoint.
*
*
***************************************************************************************/


#pragma once

#include <math.h>
#include <algorithm>
#include <cmath>
#include <chrono>


#include "DataBase/DBHandler.h"
#include "Math/CourseMath.h"
#include "Math/Utility.h"
#include "MessageBus/ActiveNode.h"
#include "Messages/ExternalControlMsg.h"
#include "Messages/StateMessage.h"
#include "Messages/WindStateMsg.h"
#include "Messages/WaypointStationKeepingMsg.h"
#include "Messages/LocalNavigationMsg.h"
#include "Messages/WingSailCommandMsg.h"
#include "Messages/RudderCommandMsg.h"
#include "SystemServices/SysClock.h"
#include "SystemServices/Timer.h"

#include "waypointrouting/RudderCommand.h"


class StationKeepingNode : public ActiveNode {
public:

	StationKeepingNode(MessageBus& msgBus, DBHandler& db);
	~StationKeepingNode();

	bool init();
	void start();
	void processMessage(const Message* message);

private:

	void processStateMessage(const StateMessage* stateMsg);

	void processWindStateMessage(const WindStateMsg* windStateMsg);

	void processWaypointMessage(const WaypointStationKeepingMsg* waypMsg);

	double computeTargetCourse();

	double computeRudder();

	bool getTargetTackStarboard(double targetCourse);

	static void StationKeepingNodeThreadFunc(ActiveNode* nodePtr);

	DBHandler &m_db;

	double  m_LoopTime;             // second	

	std::mutex m_lock;

	std::vector<float> m_TwdBuffer; // True wind direction buffer. angles in degree [0, 360[ in vessel reference frame (clockwise)

	double  m_VesselLat;
    	double  m_VesselLon;
    	double m_VesselSpeed;
    	double m_VesselHeading;

	double 	m_trueWindSpeed;		// m/s
	double 	m_trueWindDir;			// degree [0, 360[ in North-East reference frame (clockwise)

	double 	m_waypointLon;
	double 	m_waypointLat;
	int 	m_waypointRadius;	// m

	// Beating sailing mode parameters
	float  	m_CloseHauledAngle;		// radian.
	float 	m_BroadReachAngle;		// radian.
	float 	m_TackingDistance;		// meter.

	int       m_TackDirection;	

	bool     m_BeatingMode;			// True if the vessel is in beating motion (zig-zag motion).

	bool     m_stationKeeping_On;  // activate or disactivate the station keeping algorithm

	bool     m_outOfZone;  // true if we are outside the zone where we turn around

	double m_MaxRudderAngle;
};