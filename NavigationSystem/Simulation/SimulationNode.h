/****************************************************************************************
 *
 * File:
 *      SimulationNode.h
 *
 * Purpose:
 *      Discuss with the simulator via TCP.
 *      Create sensor messages from simulation data and publish them on the message bus.
 *      Listen to actuators command messages and send the command datas to the simulator.
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/

#pragma once

#include <chrono>
#include <thread>
#include <memory>
#include <stdlib.h>
#include <sys/types.h>
#include <netdb.h>
#include <fcntl.h>
#include <strings.h>
#include <cerrno>
#include <cstring>
#include <unistd.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <cmath>

#include "Math/CourseMath.h"
#include "Math/Utility.h"
#include "MessageBus/ActiveNode.h"
#include "Messages/SailCommandMsg.h"
#include "Messages/WingSailCommandMsg.h"
#include "Messages/RudderCommandMsg.h"
#include "Messages/WaypointDataMsg.h"
#include "Messages/CompassDataMsg.h"
#include "Messages/GPSDataMsg.h"
#include "Messages/MarineSensorDataMsg.h"
#include "Messages/WindDataMsg.h"
#include "SystemServices/Logger.h"
#include "SystemServices/SysClock.h"
#include "Network/TCPServer.h"
#include "WorldState/CollidableMgr/CollidableMgr.h"



enum SimulatorPacket : unsigned char {
    SailBoatData = 0, WingBoatData, AISData, CameraData, WingBoatCmd, SailBoatCmd, WaypointData, MarineSensorData};


struct SailBoatDataPacket_t {
    float latitude;
    float longitude;
    float speed;
    int16_t course;
    int16_t windDir;
    float windSpeed;
    int16_t heading;
} __attribute__((packed));

struct WingBoatDataPacket_t {
    float latitude;
    float longitude;
    float speed;
    int16_t course;
    int16_t windDir;
    float windSpeed;
    int16_t heading;
} __attribute__((packed));

struct AISContactPacket_t {
  uint16_t mmsi;
  float latitude;
  float longitude;
  float speed;
  int16_t course;
  float length;
  float beam;
} __attribute__((packed));

struct VisualFieldPacket_t {
    uint16_t relativeObstacleDistances[24];
    int16_t heading;
} __attribute__((packed));


struct ActuatorDataWingPacket_t {
    unsigned char simulatorPacket = WingBoatCmd;
    float rudderCommand;
    float tailCommand;
}__attribute__((packed));

struct ActuatorDataSailPacket_t {
    unsigned char simulatorPacket = SailBoatCmd;
    float rudderCommand;
    float sailCommand;
}__attribute__((packed));

struct WaypointPacket_t {
  unsigned char simulatorPacket = WaypointData;
  int nextId;
  double nextLongitude;
  double nextLatitude;
  int nextDeclination;
  int nextRadius;
  int nextStayTime;
  //bool isCheckpoint;
  int prevId;
  double prevLongitude;
  double prevLatitude;
  int prevDeclination;
  int prevRadius;
}__attribute__((packed));

struct MarineSensorDataPacket_t {
  unsigned char simulatorPacket = MarineSensorData;
  float temperature;
  float conductivity;
  float ph;
  float salinity;
}__attribute__((packed));


class SimulationNode : public ActiveNode {
public:
	SimulationNode(MessageBus& msgBus, bool boatType);
    SimulationNode(MessageBus& msgBus, bool boatType, CollidableMgr* collidableMgr);

    ///----------------------------------------------------------------------------------
    /// Initialize the TCP communication
    ///----------------------------------------------------------------------------------
    bool init();

    ///----------------------------------------------------------------------------------
    /// Starts the SimulationNode's thread that create all sensors messages
    ///----------------------------------------------------------------------------------
    void start();

    void processMessage(const Message* msg);

private:

    ///----------------------------------------------------------------------------------
    /// Stores sail command data from a SailCommandMsg.
    ///----------------------------------------------------------------------------------
    void processSailCommandMessage(SailCommandMsg* msg);

    ///----------------------------------------------------------------------------------
    /// Stores wing sail command data from a WingSailCommandMsg.
    ///----------------------------------------------------------------------------------
    void processWingSailCommandMessage(WingSailCommandMsg* msg);

    ///----------------------------------------------------------------------------------
    /// Stores waypoints data from a WaypointDataMsg.
    ///----------------------------------------------------------------------------------
    void processWaypointMessage(WaypointDataMsg* msg);

    ///----------------------------------------------------------------------------------
    /// Stores rudder command data from a RudderCommandMsg.
    ///----------------------------------------------------------------------------------
    void processRudderCommandMessage(RudderCommandMsg* msg);

    ///----------------------------------------------------------------------------------
    /// Stores marine sensor data from a MarineSensorDataMsg.
    ///----------------------------------------------------------------------------------
    void processMarineSensorDataMessage(MarineSensorDataMsg* msg);

    ///----------------------------------------------------------------------------------
    /// Process a conventionnal sail boat data message
    ///----------------------------------------------------------------------------------
    void processSailBoatData( TCPPacket_t& packet );

    ///----------------------------------------------------------------------------------
    /// Process a wing sail data message
    ///----------------------------------------------------------------------------------
    void processWingBoatData( TCPPacket_t& packet );

    ///----------------------------------------------------------------------------------
    /// Process a AIS contact data message
    ///----------------------------------------------------------------------------------
    void processAISContact( TCPPacket_t& packet );

    ///----------------------------------------------------------------------------------
    /// Process a visual contact data message
    ///----------------------------------------------------------------------------------
    void processVisualField( TCPPacket_t& packet );

    ///----------------------------------------------------------------------------------
    /// Send our actuators data for a wing sail-equipped boat
    ///----------------------------------------------------------------------------------
    void sendActuatorDataWing( int socketFD);

    ///----------------------------------------------------------------------------------
    /// Send our actuator data for a conventional sail-equipped boat
    ///----------------------------------------------------------------------------------
    void sendActuatorDataSail( int socketFD);

    ///----------------------------------------------------------------------------------
    /// Sends the waypoint
    ///----------------------------------------------------------------------------------
    void sendWaypoint( int socketFD );

    ///----------------------------------------------------------------------------------
    /// Sends the marine sensor data.
    ///----------------------------------------------------------------------------------
    void sendMarineSensor( int socketFD );

    ///----------------------------------------------------------------------------------
    /// Communicate with the simulation receive sensor data and send actuator data
    ///----------------------------------------------------------------------------------
    static void SimulationThreadFunc(ActiveNode* nodePtr);

    void createCompassMessage();
    void createGPSMessage();
    void createWindMessage();
    void createMarineSensorMessage(WaypointDataMsg* msg);

    float   m_RudderCommand;
    float   m_SailCommand;
    float   m_TailCommand;

    int     m_CompassHeading;
    double  m_GPSLat;
    double  m_GPSLon;
    double  m_GPSSpeed;
    double  m_GPSCourse;
    int     m_WindDir;
    float   m_WindSpeed;

    int     m_nextDeclination;  // units : degrees

    TCPServer server;
    CollidableMgr* collidableMgr;

    bool m_boatType;

    ActuatorDataWingPacket_t actuatorDataWing;
    ActuatorDataSailPacket_t actuatorDataSail;
    WaypointPacket_t waypoint;
    MarineSensorDataPacket_t marineSensorData;

    std::mutex m_lock;
};
