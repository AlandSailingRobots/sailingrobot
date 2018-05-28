#pragma once

#include "DataBase/DBLogger.h"
#include "MessageBus/ActiveNode.h"
#include "MessageBus/MessageBus.h"
#include "MessageBus/MessageTypes.h"

#include <iostream>
#include <mutex>

class DBLoggerNode : public ActiveNode {
   public:
    DBLoggerNode(MessageBus& msgBus, DBHandler& db, int queueSize);

    void processMessage(const Message* message);

    void updateConfigsFromDB();

    void start();

    void stop();

    bool init();

   private:
    static void DBLoggerNodeThreadFunc(ActiveNode* nodePtr);

    int DATA_OUT_OF_RANGE = -2000;

    DBHandler& m_db;
    DBLogger m_dbLogger;

    // struct used from DBHandler.h
    LogItem item{
        (double)DATA_OUT_OF_RANGE,   // m_rudderPosition;
        (double)DATA_OUT_OF_RANGE,   // m_wingsailPosition;
        (bool)false,                 // m_radioControllerOn;
        (double)DATA_OUT_OF_RANGE,   // m_windVaneAngle;
        (double)DATA_OUT_OF_RANGE,   // m_compassHeading;
        (double)DATA_OUT_OF_RANGE,   // m_compassPitch;
        (double)DATA_OUT_OF_RANGE,   // m_compassRoll;
        (double)DATA_OUT_OF_RANGE,   // m_distanceToWaypoint;
        (double)DATA_OUT_OF_RANGE,   // m_bearingToWaypoint;
        (double)DATA_OUT_OF_RANGE,   // m_courseToSteer;
        (bool)false,                 // m_tack;
        (bool)false,                 // m_goingStarboard;
        (double)DATA_OUT_OF_RANGE,   // m_currentActuatorUnit;
        (double)DATA_OUT_OF_RANGE,   // m_currentNavigationUnit;
        (double)DATA_OUT_OF_RANGE,   // m_currentWindVaneAngle;
        (double)DATA_OUT_OF_RANGE,   // m_currentWindVaneClutch;
        (double)DATA_OUT_OF_RANGE,   // m_currentSailboatDrive;
        (bool)false,                 // m_gpsHasFix;
        (bool)false,                 // m_gpsOnline;
        (double)DATA_OUT_OF_RANGE,   // m_gpsLat;
        (double)DATA_OUT_OF_RANGE,   // m_gpsLon;
        (double)DATA_OUT_OF_RANGE,   // m_gpsUnixTime;
        (double)DATA_OUT_OF_RANGE,   // m_gpsSpeed;
        (double)DATA_OUT_OF_RANGE,   // m_gpsCourse;
        (int)DATA_OUT_OF_RANGE,      // m_gpsSatellite;
        (bool)false,                 // m_routeStarted;
        (float)DATA_OUT_OF_RANGE,    // m_temperature;
        (float)DATA_OUT_OF_RANGE,    // m_conductivity;
        (float)DATA_OUT_OF_RANGE,    // m_ph;
        (float)DATA_OUT_OF_RANGE,    // m_salinity;
        (double)DATA_OUT_OF_RANGE,   // m_vesselHeading;
        (double)DATA_OUT_OF_RANGE,   // m_vesselLat;
        (double)DATA_OUT_OF_RANGE,   // m_vesselLon;
        (double)DATA_OUT_OF_RANGE,   // m_vesselSpeed;
        (double)DATA_OUT_OF_RANGE,   // m_vesselCourse;
        (double)DATA_OUT_OF_RANGE,   // m_trueWindSpeed;
        (double)DATA_OUT_OF_RANGE,   // m_trueWindDir;
        (double)DATA_OUT_OF_RANGE,   // m_apparentWindSpeed;
        (double)DATA_OUT_OF_RANGE,   // m_apparentWindDir;
        (float)DATA_OUT_OF_RANGE,    // m_windSpeed;
        (float)DATA_OUT_OF_RANGE,    // m_windDir;
        (float)DATA_OUT_OF_RANGE,    // m_windTemp;
        (std::string) "initialized"  // m_timestamp_str;
    };

    double m_loopTime;
    int m_queueSize;

    std::mutex m_lock;
    std::atomic<bool> m_Running;
};
