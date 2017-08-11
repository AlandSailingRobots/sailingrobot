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

#include "DataBase/DBHandler.h"
#include "MessageBus/ActiveNode.h"
#include "Messages/CompassDataMsg.h"
#include "Messages/GPSDataMsg.h"
#include "Messages/WindDataMsg.h"
#include "Messages/ArduinoDataMsg.h"
#include "Messages/WaypointDataMsg.h"
#include "DataBase/DBHandler.h"
#include "Network/TCPServer.h"
#include <mutex>
#include <stdint.h>
#include <atomic>

class StateEstimationNode : public ActiveNode {
public:
  StateEstimationNode(MessageBus& msgBus, DBHandler& dbhandler, double loopTime);
  ~StateEstimationNode();

  bool init();

  ///----------------------------------------------------------------------------------
  /// Starts the StateEstimationNode's thread that pumps out StateMessages
  ///----------------------------------------------------------------------------------
  void start();

  void stop();



  void processMessage(const Message* msg);


private:

  ///----------------------------------------------------------------------------------
  /// Update values from the database as the loop time of the thread and others parameters
  ///----------------------------------------------------------------------------------
  void updateConfigsFromDB();

  ///----------------------------------------------------------------------------------
  /// Stores compass data from a CompassDataMsg.
  ///----------------------------------------------------------------------------------
  void processCompassMessage(const CompassDataMsg* msg);
  ///----------------------------------------------------------------------------------
  /// Stores the GPS data from a GPSDataMsg.
  ///----------------------------------------------------------------------------------
  void processGPSMessage(const GPSDataMsg* msg);
  void processWaypointMessage(const WaypointDataMsg* msg );

  int getCourse();
  ///----------------------------------------------------------------------------------
  /// Starts the StateEstimationNode's thread that pumps out StateMessages which contains
  /// data collected from the sensors
  ///----------------------------------------------------------------------------------
  static void StateEstimationNodeThreadFunc(ActiveNode* nodePtr);

  float 	m_VesselHeading;
  double	m_VesselLat;
  double	m_VesselLon;
  double	m_VesselSpeed;
  double    m_VesselCourse;

  double    m_LoopTime;
  int       m_Declination;

  double    m_SpeedLimit;
  bool      m_GpsOnline;
  const int STATE_INITIAL_SLEEP = 2000;

  std::mutex            m_lock;
  std::atomic<bool>     m_Running;
  DBHandler&            m_dbHandler;

};
