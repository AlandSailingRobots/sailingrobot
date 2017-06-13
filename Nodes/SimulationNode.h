/****************************************************************************************
 *
 * File:
 * 		SimulationNode.h
 *
 * Purpose:
 *		Discuss with simulation via TCP, create message for the program from the
 *    data from simulation and send the command data to the simulation.
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/

#pragma once


#include <sys/socket.h>
#include <arpa/inet.h>

#include "MessageBus/ActiveNode.h"
#include "Messages/CompassDataMsg.h"
#include "Messages/GPSDataMsg.h"
#include "Messages/WindDataMsg.h"
#include "Messages/ArduinoDataMsg.h"
#include "Messages/ActuatorPositionMsg.h"
#include "Network/TCPServer.h"
#include "CollidableMgr/CollidableMgr.h"


struct BoatDataPacket_t {
  float latitude;
  float longitude;
  float speed;
  uint16_t course;

  uint16_t windDir;
  float windSpeed;

  uint16_t heading;
  uint16_t rudder;
  uint16_t sail;
} __attribute__((packed));

struct AISContactPacket_t {
  uint32_t mmsi;
  float latitude;
  float longitude;
  float speed;
  uint16_t course;
} __attribute__((packed));

struct VisualContactPacket_t {
  uint32_t id;
  float latitude;
  float longitude;
} __attribute__((packed));

struct ActuatorDataPacket_t {
  uint16_t rudderCommand;
  uint16_t sailCommand;
}__attribute__((packed));


class SimulationNode : public ActiveNode {
public:
	SimulationNode(MessageBus& msgBus);
  SimulationNode(MessageBus& msgBus, CollidableMgr* collidableMgr);

	///----------------------------------------------------------------------------------
	/// Initialize the TCP communication
	///----------------------------------------------------------------------------------
	bool init();

	///----------------------------------------------------------------------------------
 	/// Starts the SimulationNode's thread that create all sensors messages
 	///----------------------------------------------------------------------------------
	void start();

	void processMessage(const Message* msg);

	///----------------------------------------------------------------------------------
	/// Stores compass data from a ActuatorPositionMsg.
	///----------------------------------------------------------------------------------
	void processActuatorPositionMessage(ActuatorPositionMsg* msg);

private:

  ///----------------------------------------------------------------------------------
  /// Process a boat data message
  ///----------------------------------------------------------------------------------
  void processBoatData( TCPPacket_t& packet );

	///----------------------------------------------------------------------------------
	/// Communicate with the simulation receive sensor data and send actuator data
	///----------------------------------------------------------------------------------
	static void SimulationThreadFunc(ActiveNode* nodePtr);

  ///----------------------------------------------------------------------------------
  /// Process a AIS contact data message
  ///----------------------------------------------------------------------------------
  void processAISContact( TCPPacket_t& packet );

  ///----------------------------------------------------------------------------------
  /// Process a visual contact data message
  ///----------------------------------------------------------------------------------
  void processVisualContact( TCPPacket_t& packet );

  ///----------------------------------------------------------------------------------
  /// Send our actuator data
  ///----------------------------------------------------------------------------------
  void sendActuatorData( int socketFD );

  ///----------------------------------------------------------------------------------
	/// Communicate with the simulation receive sensor data and send actuator data
	///----------------------------------------------------------------------------------
	//static void SimulationThreadFunc(void* nodePtr);

  void createCompassMessage();
  void createGPSMessage();
  void createWindMessage();
  void createArduinoMessage();

	int 	  m_CompassHeading;
	double	m_GPSLat;
	double	m_GPSLon;
	double	m_GPSSpeed;
	double	m_GPSHeading;
	int	    m_WindDir;
	float	  m_WindSpeed;
	int 	  m_ArduinoRudder;
	int 	  m_ArduinoSheet;

  ActuatorDataPacket_t actuatorData;
  TCPServer server;
  CollidableMgr* collidableMgr;

};
