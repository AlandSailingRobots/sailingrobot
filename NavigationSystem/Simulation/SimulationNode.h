/****************************************************************************************
 *
 * File:
 *      SimulationNode.h
 *
 * Purpose:
 *      Discuss with simulation via TCP, create message for the program from the
 *    data from simulation and send the command data to the simulation.
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/

#pragma once


#include <sys/socket.h>
#include <arpa/inet.h>

#include "DataBase/DBHandler.h"
#include "MessageBus/ActiveNode.h"
#include "Messages/CompassDataMsg.h"
#include "Messages/GPSDataMsg.h"
#include "Messages/WindDataMsg.h"
#include "Messages/SailCommandMsg.h"
#include "Messages/ASPireActuatorFeedbackMsg.h"
#include "Messages/WingSailCommandMsg.h"
#include "Messages/RudderCommandMsg.h"
#include "Messages/WaypointDataMsg.h"

#include "Network/TCPServer.h"
#include "WorldState/CollidableMgr/CollidableMgr.h"


enum SimulatorPacket : unsigned char {
    SailBoatData = 0,
    WingBoatData =1,
    AISData = 2,
    CameraData = 3,
    WingBoatCmd = 4,
    SailBoatCmd = 5,
    WaypointData = 6
};

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
  uint32_t mmsi;
  float latitude;
  float longitude;
  float speed;
  int16_t course;
  float length;
  float beam;
} __attribute__((packed));

struct VisualContactPacket_t {
    uint32_t id;
    float latitude;
    float longitude;
} __attribute__((packed));

struct ActuatorDataWingPacket_t {
    unsigned char simulatorPacket = WingBoatCmd;
    float rudderCommand;
    float tailCommand;
}__attribute__((packed));

struct ActuatorDataSailPacket_t
{
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
  int prevId;
  double prevLongitude;
  double prevLatitude;
  int prevDeclination;
  int prevRadius;
}__attribute__((packed));


class SimulationNode : public ActiveNode {
public:
	SimulationNode(MessageBus& msgBus, DBHandler& dbhandler, bool boatType);
    SimulationNode(MessageBus& msgBus, DBHandler& dbhandler, bool boatType, CollidableMgr* collidableMgr);

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
    /// Stores sail command data from a SailCommandMsg.
    ///----------------------------------------------------------------------------------
    void processSailCommandMessage(SailCommandMsg* msg);

    ///----------------------------------------------------------------------------------
    /// Stores wing sail command data from a WingSailCommandMsg.
    ///----------------------------------------------------------------------------------
    void processWingSailCommandMessage(WingSailCommandMsg* msg);


    void processWaypointMessage(WaypointDataMsg* msg);

    ///----------------------------------------------------------------------------------
    /// Stores rudder command data from a RudderCommandMsg.
    ///----------------------------------------------------------------------------------
    void processRudderCommandMessage(RudderCommandMsg* msg);

private:

    ///----------------------------------------------------------------------------------
    /// Update values from the database as the loop time of the thread and others parameters
    ///----------------------------------------------------------------------------------
    void updateConfigsFromDB();

    ///----------------------------------------------------------------------------------
    /// Process a conventionnal sail boat data message
    ///----------------------------------------------------------------------------------
    void processSailBoatData( TCPPacket_t& packet );

    ///----------------------------------------------------------------------------------
    /// Process a wing sail data message
    ///----------------------------------------------------------------------------------
    void processWingBoatData( TCPPacket_t& packet );

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
    /// Communicate with the simulation receive sensor data and send actuator data
    ///----------------------------------------------------------------------------------
    //static void SimulationThreadFunc(void* nodePtr);

    void createCompassMessage();
    void createGPSMessage();
    void createWindMessage();

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
    DBHandler& m_db;

    bool m_boatType;

    ActuatorDataWingPacket_t actuatorDataWing;
    ActuatorDataSailPacket_t actuatorDataSail;
    WaypointPacket_t waypoint;

    std::mutex m_lock;
};
