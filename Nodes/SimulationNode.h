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
	/// Communicate with the simulation receive sensor data and send actuator data
	///----------------------------------------------------------------------------------
	static void SimulationThreadFunc(void* nodePtr);

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

  void createCompassMessage();
  void createGPSMessage();
  void createWindMessage();
  void createArduinoMessage();

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
  int   m_rudder;
  int   m_sail;

  struct DATA_SOCKET_RECEIVE m_data_receive;
  struct DATA_SOCKET_SEND m_data_send;
  struct HANDLERS_SOCKET m_handler_socket_server;
  struct HANDLERS_SOCKET m_handler_socket_client;

  int m_count_sleep;

};
