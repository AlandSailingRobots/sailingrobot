/****************************************************************************************
*
* File:
* 		StateEstimationNode.h
*
*  Purpose:
*		Maintains the "current" state listening to GPS and compass messages, sending out a
* StateMessage
*
* Developer Notes:
*
*
***************************************************************************************/

#pragma once

#include "ActiveNode.h"
#include "Messages/CompassDataMsg.h"
#include "Messages/GPSDataMsg.h"
#include "Messages/WindDataMsg.h"
#include "Messages/ArduinoDataMsg.h"
#include "Messages/WaypointDataMsg.h"
#include "Network/TCPServer.h"
#include <stdint.h>

class StateEstimationNode : public ActiveNode {
public:
  StateEstimationNode(MessageBus& msgBus, double loopTime);
  ~StateEstimationNode();

  ///----------------------------------------------------------------------------------
  /// Initialises the server.
  ///----------------------------------------------------------------------------------
  bool init();

  ///----------------------------------------------------------------------------------
  /// Starts the StateEstimationNode's thread that pumps out StateMessages
  ///----------------------------------------------------------------------------------
  void start();

  void processMessage(const Message* msg);

  ///----------------------------------------------------------------------------------
  /// Stores compass data from a CompassDataMsg.
  ///----------------------------------------------------------------------------------
  void processCompassMessage(CompassDataMsg* msg);

  ///----------------------------------------------------------------------------------
  /// Stores the GPS data from a GPSDataMsg.
  ///----------------------------------------------------------------------------------
  void processGPSMessage(GPSDataMsg* msg);
  void processWaypointMessage( WaypointDataMsg* msg );


private:
  ///----------------------------------------------------------------------------------
  /// Starts the StateEstimationNode's thread that pumps out StateMessages which contains
  /// data collected from the sensors
  ///----------------------------------------------------------------------------------
  static void StateEstimationNodeThreadFunc(void* nodePtr);

  float 	vesselHeading;
  double	vesselLat;
  double	vesselLon;
  double	vesselSpeed;

  double loopTime;
  int declination;

  TCPServer server;
};
