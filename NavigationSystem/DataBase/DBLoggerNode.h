#pragma once

#include "DataBase/DBLogger.h"
#include "MessageBus/ActiveNode.h"
#include "MessageBus/MessageTypes.h"
#include "MessageBus/MessageBus.h"

#include <mutex>
#include <iostream>

class DBLoggerNode: public ActiveNode {
public:
    DBLoggerNode(MessageBus& msgBus, DBHandler& db, int TimeBetweenMsgs, int updateFrequency, int queueSize);

    void processMessage(const Message* message);

    void start();

    bool init();

private:

    static void DBLoggerNodeThreadFunc(ActiveNode* nodePtr);

    int DATA_OUT_OF_RANGE = -2000;

    DBHandler &m_db;
	DBLogger m_dbLogger;

// struct used from DBHandler.h
    LogItem item {
     (int)      DATA_OUT_OF_RANGE,  // m_compassHeading;
     (int)      DATA_OUT_OF_RANGE,  // m_compassPitch;
     (int)      DATA_OUT_OF_RANGE,  // m_compassRoll;
     (bool)     false,              // m_gpsHasFix;
     (bool)     false,              // m_gpsOnline;
     (double)   DATA_OUT_OF_RANGE,  // m_gpsLat;
     (double)   DATA_OUT_OF_RANGE,  // m_gpsLon;
     (double)   DATA_OUT_OF_RANGE,  // m_gpsUnixTime;
     (double)   DATA_OUT_OF_RANGE,  // m_gpsSpeed;
     (double)   DATA_OUT_OF_RANGE,  // m_gpsHeading;
     (int)      DATA_OUT_OF_RANGE,  // m_gpsSatellite;
     (float)    DATA_OUT_OF_RANGE,  // m_windSpeed;
     (float)    DATA_OUT_OF_RANGE,  // m_windDir;
     (float)    DATA_OUT_OF_RANGE,  // m_windTemp;
     (int)      DATA_OUT_OF_RANGE,  // m_arduinoPressure;
     (int)      DATA_OUT_OF_RANGE,  // m_arduinoRudder;
     (int)      DATA_OUT_OF_RANGE,  // m_arduinoSheet;
     (int)      DATA_OUT_OF_RANGE,  // m_arduinoBattery;
     (double)   DATA_OUT_OF_RANGE,  // m_rudder;
     (double)   DATA_OUT_OF_RANGE,  // m_sail;
     (int)      DATA_OUT_OF_RANGE,  // m_sailServoPosition;
     (int)      DATA_OUT_OF_RANGE,  // m_rudderServoPosition;
     (double)   DATA_OUT_OF_RANGE,  // m_distanceToWaypoint;
     (double)   DATA_OUT_OF_RANGE,  // m_bearingToWaypoint;
     (double)   DATA_OUT_OF_RANGE,  // m_courseToSteer;
     (bool)     false,              // m_tack;
     (bool)     false,              // m_goingStarboard;
     (int)      DATA_OUT_OF_RANGE,  // m_waypointId;
     (double)   DATA_OUT_OF_RANGE,  // m_twd;
     (bool)     false,              // m_routeStarted;
     (std::string) "initialized"    // m_timestamp_str;
    };

    int m_TimeBetweenMsgs;
    int m_updateFrequency;
    int m_queueSize;

    std::mutex m_lock;

};
