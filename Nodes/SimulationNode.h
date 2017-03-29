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

#include "ActiveNode.h"
#include "Messages/CompassDataMsg.h"
#include "Messages/GPSDataMsg.h"
#include "Messages/WindDataMsg.h"
#include "Messages/ArduinoDataMsg.h"
#include "Messages/ActuatorPositionMsg.h"
#include "Network/TCPServer.h"

//-----------------------------------------------------------------------------
struct DATA_SOCKET_RECEIVE{

  //=========================
  float latitude=0;
  float longitude=0;
  float course_real = 0;
  float course_magn = 0;
  float speed_knot = 0;

  //=========================
  float windDirection = 120;
  float windSpeed = 2.1;
  float windTemperature = 24;

  //=========================
  uint16_t pressure;
  uint16_t rudder;
  uint16_t sheet;
  uint16_t battery;


  //=========================

  uint16_t headingVector[3];
  uint16_t magVector[3];
  uint16_t tiltVector[3];
  uint16_t accelVector[3];
  uint8_t address_compass;
  uint8_t address_arduino;
}__attribute__((packed));

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

struct ActuatorDataPacket_t {
  uint16_t rudderCommand;
  uint16_t sailCommand;
}__attribute__((packed));


struct DATA_SOCKET_SEND{
  uint16_t rudder_command;
  uint16_t sheet_command;
}__attribute__((packed));
//-----------------------------------------------------------------------------

struct HANDLERS_SOCKET
{
    int sockfd;
    struct sockaddr_in info_me;
};

class SimulationNode : public ActiveNode {
public:
	SimulationNode(MessageBus& msgBus);

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
  /// Initalize socket server
  ///----------------------------------------------------------------------------------
  int init_socket(int port);

  ///----------------------------------------------------------------------------------
  /// Manage data received from simulation
  ///----------------------------------------------------------------------------------
  void processSocketData();

  ///----------------------------------------------------------------------------------
  /// Manage data to send to simulation
  ///----------------------------------------------------------------------------------
  void setupDataSend();

  ///----------------------------------------------------------------------------------
  /// Process a boat data message
  ///----------------------------------------------------------------------------------
  void processBoatData( TCPPacket_t& packet );

  ///----------------------------------------------------------------------------------
  /// Send our actuator data
  ///----------------------------------------------------------------------------------
  void sendActuatorData( int socketFD );

  ///----------------------------------------------------------------------------------
	/// Communicate with the simulation receive sensor data and send actuator data
	///----------------------------------------------------------------------------------
	static void SimulationThreadFunc(void* nodePtr);

  void createCompassMessage();
  void createGPSMessage();
  void createWindMessage();
  void createArduinoMessage();

	int 	  m_CompassHeading;
	double	m_GPSLat;
	double	m_GPSLon;
	double	m_GPSUnixTime;
	double	m_GPSSpeed;
	double	m_GPSHeading;
	int	    m_WindDir;
	float	  m_WindSpeed;
	int 	  m_ArduinoRudder;
	int 	  m_ArduinoSheet;

  int     m_rudder;
  int     m_sail;
  ActuatorDataPacket_t actuatorData;

  struct DATA_SOCKET_RECEIVE m_data_receive;
  struct DATA_SOCKET_SEND m_data_send;
  struct HANDLERS_SOCKET m_handler_socket_server;
  struct HANDLERS_SOCKET m_handler_socket_client;

  int m_count_sleep;

  TCPServer server;

};
