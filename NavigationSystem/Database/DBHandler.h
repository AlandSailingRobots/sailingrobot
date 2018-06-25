#pragma once

#include <sqlite3.h>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include "../Messages/CurrentSensorDataMsg.h"
#include "../Messages/WindStateMsg.h"
#include "../SystemServices/Logger.h"

//#include <include/nlohmann/json.hpp>
#include "../Libs/json/include/nlohmann/json.hpp"
using Json = nlohmann::json;

struct LogItem {
    double m_rudderPosition;  // dataLogs_actuator_feedback
    double m_wingsailPosition;
    bool m_radioControllerOn;
    double m_windVaneAngle;
    double m_compassHeading;  // dataLogs_compass
    double m_compassPitch;
    double m_compassRoll;
    double m_distanceToWaypoint;  // dataLogs_course_calculation
    double m_bearingToWaypoint;
    double m_courseToSteer;
    bool m_tack;
    bool m_goingStarboard;
    bool m_gpsHasFix;  // dataLogs_gps
    bool m_gpsOnline;
    double m_gpsLat;
    double m_gpsLon;
    double m_gpsUnixTime;
    double m_gpsSpeed;
    double m_gpsCourse;
    int m_gpsSatellite;
    bool m_routeStarted;
    float m_temperature;  // dataLogs_marine_sensors
    float m_conductivity;
    float m_ph;
    float m_salinity;
    double m_vesselHeading;  // dataLogs_vessel_state
    double m_vesselLat;
    double m_vesselLon;
    double m_vesselSpeed;
    double m_vesselCourse;
    double m_trueWindSpeed;  // dataLogs_wind_state
    double m_trueWindDir;
    double m_apparentWindSpeed;
    double m_apparentWindDir;
    float m_windDir;  // dataLogs_windsensor
    float m_windSpeed;
    float m_windTemp;
    float m_current;  // dataLogs_current_sensors
    float m_voltage;
    SensedElement m_element;
    std::string m_element_str;
    std::string m_timestamp_str;
};

class DBHandler {
   private:
    char* m_error;
    int m_latestDataLogId;
    std::string m_currentWaypointId = "";
    std::string m_filePath;
    static std::mutex m_databaseLock;
    sqlite3 *m_DBHandle = NULL;

    // execute INSERT query and add new row into table
    bool queryTable(std::string sqlINSERT);
    bool queryTable(std::string sqlINSERT, sqlite3* db);

    // retrieve data from given table/tables, return value is a C 2D char array
    // rows and columns also return values (through a reference) about rows and columns in the
    // result set
    std::vector<std::string> retrieveFromTable(std::string sqlSELECT, int& rows, int& columns);
    std::vector<std::string> retrieveFromTable(std::string sqlSELECT,
                                               int& rows,
                                               int& columns,
                                               sqlite3* db);

    // adds a table row into the json object as a array if array flag is true,
    // otherwise it adds the table row as a json object
    // id field is not obligatory, can be left empty
    void getDataAsJson(std::string select,
                       std::string table,
                       std::string key,
                       std::string id,
                       Json& js,
                       bool useArray);

    // gets the id column from a given table
    std::vector<std::string> getTableIds(std::string table);

    // gets all datatable names related to "ending" string
    // used to fetch all tables ending with _datalogs or _config
    std::vector<std::string> getTableNames(std::string like);

    // gets information(for instance: name/datatype) about all columns
    std::vector<std::string> getColumnInfo(std::string info, std::string table);

    // help function used in insertDataLog
    int insertLog(std::string table, std::string values, sqlite3* db);

    // own implementation of deprecated sqlite3_get_table()
    int getTable(sqlite3* db,
                 const std::string& sql,
                 std::vector<std::string>& results,
                 int& rows,
                 int& columns);

    sqlite3* DBConnect();
    void DBDisconnect();
    void DBClose();

   public:
    DBHandler(std::string filePath);
    ~DBHandler(void);

    bool initialise();

    int getRows(std::string table);

    void insertDataLogs(std::vector<LogItem>& logs);

    void insertMessageLog(std::string gps_time, std::string type, std::string msg);

    // updates table with json string (data)
    bool updateTableJson(std::string table, std::string data);
    bool updateTableJsonObject(std::string table, Json data);

    // updates table using values given
    bool updateTable(std::string table, std::string column, std::string value, std::string id);

    void clearTable(std::string table);

    void updateConfigs(std::string configs);
    bool updateWaypoints(std::string waypoints);

    // retrieve one value from a table as string
    std::string retrieveCell(std::string table, std::string id, std::string column);

    // retrieve one value from a table as integer
    int retrieveCellAsInt(std::string table, std::string id, std::string column);

    // retrieve one value from a table as double
    double retrieveCellAsDouble(std::string table, std::string id, std::string column);

    // returns all logs in database as json; supply onlyLatest to get only the ones with the highest
    // id
    std::string getLogs(bool onlyLatest);

    void forceUnlock() { m_databaseLock.unlock(); }

    void clearLogs();

    // get id from table returns either max or min id from table.
    // max = false -> min id
    // max = true -> max id
    std::string getIdFromTable(std::string table, bool max);
    std::string getIdFromTable(std::string table, bool max, sqlite3* db);

    void deleteRow(std::string table, std::string id);

    bool getWaypointValues(int& nextId,
                           double& nextLongitude,
                           double& nextLatitude,
                           int& nextDeclination,
                           int& nextRadius,
                           int& nextStayTime,
                           bool& isCheckpoint,
                           int& prevId,
                           double& prevLongitude,
                           double& prevLatitude,
                           int& prevDeclination,
                           int& prevRadius,
                           bool& foundPrev);

    bool insert(std::string table, std::string fields, std::string values);
    bool changeOneValue(std::string table,
                        std::string id,
                        std::string newValue,
                        std::string colName);

    std::string getWaypoints();

    std::string getConfigs();
};
