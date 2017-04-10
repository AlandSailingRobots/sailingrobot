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
  StateEstimationNode(MessageBus& msgBus);
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

private:
  ///----------------------------------------------------------------------------------
  /// Starts the StateEstimationNode's thread that pumps out StateMessages which contains
  /// data collected from the sensors
  ///----------------------------------------------------------------------------------
  static void StateEstimationNodeThreadFunc(void* nodePtr);

  int 	m_CompassHeading;
  int 	m_CompassPitch;
  int 	m_CompassRoll;
  bool	m_GPSHasFix;
  bool	m_GPSOnline;
  double	m_GPSLat;
  double	m_GPSLon;
  double	m_GPSUnixTime;
  double	m_GPSSpeed;
  double	m_GPSHeading;
  int		m_GPSSatellite;

  int16_t radius;


  TCPServer server;
};
