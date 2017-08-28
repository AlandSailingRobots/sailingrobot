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


#include "Math/CourseMath.h"
#include "Math/Utility.h"
#include "MessageBus/ActiveNode.h"
#include "Messages/ExternalControlMsg.h"
#include "Messages/StateMessage.h"
#include "Messages/WindStateMsg.h"
#include "Messages/WaypointStationKeepingMsg.h"
#include "Messages/LocalNavigationMsg.h"
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

	void processWaypointMessage(WaypointStationKeepingMsg* waypMsg);

	static void LineFollowNodeThreadFunc(ActiveNode* nodePtr);