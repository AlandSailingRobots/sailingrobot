/****************************************************************************************
*
* File:
* 		StateEstimationNode.h
*
*  Purpose:
*		Maintains the "current" listening to GPS and compass messages, sending out a
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
  float	m_WindDir;
  float	m_WindSpeed;
  float 	m_WindTemp;
  int 	m_ArduinoPressure;
  int 	m_ArduinoRudder;
  int 	m_ArduinoSheet;
  int 	m_ArduinoBattery;
  int		m_ArduinoRC;

  int waypointID;
  double waypointLat;
  double waypointLon;
  int16_t waypointBearing;
  double waypointDistance;
  int16_t radius;


  TCPServer server;
};
