/**************************************************************************
 *
 * File:
 * 		DBHandler.cpp
 *
 * Purpose:
 *		Interacts with the SQLite database
 *
 * Developer Notes:
 *		Refactored 2018-07 by Kåre Hampf <khampf@users.sourceforge.net>
 *
 ***************************************************************************************/

#pragma once

#include <sqlite3.h>
#include <iostream>
#include <mutex>
#include <queue>
#include <string>
#include <vector>
#include <nlohmann/json.hpp>
#include "../Messages/CurrentSensorDataMsg.h"
#include "../Messages/WindStateMsg.h"
using JSON = nlohmann::json;

struct currentSensorItem {
	float m_current;  // dataLogs_current_sensors
	float m_voltage;
	SensedElement m_element;
	std::string m_element_str;
};

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
	std::queue<currentSensorItem> m_currentSensorItems;
	std::string m_timestamp_str;
};

class DBHandler {
    /*******************************************************************************
     * private
     ******************************************************************************/
   private:
    std::mutex m_databaseLock;
    char* m_error;
    int m_latestDataLogId;
    // std::string m_currentWaypointId = "";
    std::string m_filePath;
    sqlite3* m_DBHandle = nullptr;

    // Reusable statements for dataLog inserts
    sqlite3_stmt* m_actuatorFeedbackStmt = nullptr;
    sqlite3_stmt* m_compassModelStmt = nullptr;
    sqlite3_stmt* m_courseCalculationStmt = nullptr;
    sqlite3_stmt* m_marineSensorsStmt = nullptr;
    sqlite3_stmt* m_vesselStateStmt = nullptr;
    sqlite3_stmt* m_windStateStmt = nullptr;
    sqlite3_stmt* m_windsensorStmt = nullptr;
    sqlite3_stmt* m_gpsStmt = nullptr;
    sqlite3_stmt* m_currentSensorsStmt = nullptr;
    sqlite3_stmt* m_systemStmt = nullptr;

    // execute INSERT query and add new row into table
    bool DBTransaction(const std::string& SQLQuery);
    // bool DBTransaction(std::string SQLQuery, sqlite3 *db);

    // gets information(for instance: name/datatype) about all columns
    // std::vector<std::string> getColumnInfo(std::string& info, std::string& table);

    /*
        // retrieve data from given table/tables, return value is a C 2D char array
        // rows and columns also return values (through a reference) about rows and columns in the
        // result set
        std::vector<std::string> retrieveFromTable(const std::string& SQLSelect,
                                                   int& rows,
                                                   int& columns);
        // own implementation of deprecated sqlite3_get_table()
        int getTable(const std::string& sql,
                     std::vector<std::string>& results,
                     int& rows,
                     int& columns);
    */

    sqlite3* DBConnect();
    void DBDisconnect();
    void DBClose();

    // Internal data structures
    typedef std::vector<std::string> textTableRow;
    typedef std::vector<textTableRow>
        textTableRows;  // 	typedef std::vector<std::vector<std::string>>;
    typedef std::pair<std::string, textTableRows>
        textTable;  // std::pair<std::string, std::vector<std::vector<std::string>>>
    typedef std::vector<textTable>
        textTables;  // std::vector<std::pair<std::string, std::vector<std::vector<std::string>>>>
    typedef std::vector<std::pair<std::string, int>> ColumnTypes;
    // For binding parameter values
    struct typedValuePairs {
        std::vector<std::pair<std::string, int>> ints;
        std::vector<std::pair<std::string, double>> doubles;
        std::vector<std::pair<std::string, std::string>> strings;
    };
    typedef std::vector<typedValuePairs> tableRows;

	// gets all database table names related to "suffix" string
	// used to fetch all tables ending with _datalogs or _config
	std::vector<std::string> getTableNames(const std::string &like, const std::string &statements = "");
	int sqliteTypeFromString(std::string const& typestr);
	ColumnTypes getTableColumnTypes(const std::string& tableName);
	std::vector<std::string> getTableColumnNames(const std::string& tableName);
	int columnType(const std::string& name, ColumnTypes& types);

	bool JSONAsTables(const std::string& string, textTables& tables);
    void clearTable(const std::string& table);
    /* DBHandler:: */ textTables getTablesAsText(const std::string& like,
                                                 const std::string& statement = "");
    std::string getTablesAsJSON(const std::string& like, const std::string& statement = "");
    // For preparing
    int prepareStmtError(sqlite3_stmt*& stmt, const std::string& sql);  // Ref here gave segfaults
    int prepareStmtSelectFromStatements(sqlite3_stmt*& stmt,
                                        const std::string& expressions,
                                        const std::string& tables,
                                        const std::string& statements = nullptr);
    int paramNameIndex(sqlite3_stmt*& stmt, std::string name);

    int bindValuesToStmt(const typedValuePairs& values, sqlite3_stmt*& stmt);
    void addValue(typedValuePairs& values, const std::string& name, int value);
    void addValue(typedValuePairs& values, const std::string& name, double value);
    void addValue(typedValuePairs& values, const std::string& name, std::string& string);
    std::vector<std::string> valueNames(const typedValuePairs& values);
    void valuesFromTextRows(tableRows& values,
                            const textTableRows& textRows,
                            const ColumnTypes& types);
    int bindParam(sqlite3_stmt*& stmt, const std::string& name, int value);
    int bindParam(sqlite3_stmt*& stmt, const std::string& name, double value);
    int bindParam(sqlite3_stmt*& stmt, const std::string& name, const std::string& text);

    // INSERT
    int prepareStmtInsertError(sqlite3_stmt*& stmt,
                               const std::string& table,
                               std::vector<std::string>& columns);
    int prepareStmtInsertError(sqlite3_stmt*& stmt,
                               const std::string& table,
                               const typedValuePairs& values);
    bool insertTableRow(const std::string& tableName, const typedValuePairs& values);
    int insertTableRowsErrors(const std::string& tableName, const tableRows& rows);

    bool transactionalReplaceTable(const std::string& tableName, const tableRows& rows);
    bool transactionalReplaceTable(const std::string& tableName, const textTableRows& rows);
    bool transactionalReplaceTable(const textTable& table);
    bool replaceTables(const textTables& tables);

    // UPDATE
    int prepareStmtUpdateError(sqlite3_stmt*& stmt,
                               const std::string& table,
                               const int id,
                               const std::vector<std::string>& columns);
    int prepareStmtUpdateError(sqlite3_stmt*& stmt,
                               const std::string& table,
                               const int id,
                               const typedValuePairs& values);
    bool updateTableRow(const std::string& table, int id, const typedValuePairs& values);

    // Private SQLite wrapper functions
    int checkRetCode(int retCode) const;
    int stepAndFinalizeStmt(sqlite3_stmt*& stmt) const;
    void sqlite3_column_value(sqlite3_stmt*& stmt, int index, int& value);
    void sqlite3_column_value(sqlite3_stmt*& stmt, int index, bool& value);
    void sqlite3_column_value(sqlite3_stmt*& stmt, int index, double& value);
    void sqlite3_column_value(sqlite3_stmt*& stmt, int index, std::string& value);

    template <typename T>
    int refSelectFromTemplate(T& ref,
                              const typedValuePairs& values,
                              const std::string& selector,
                              const std::string& from,
                              const std::string& statements = nullptr) {
        int retCode;
        sqlite3_stmt* stmt = nullptr;

        retCode = prepareStmtSelectFromStatements(stmt, selector, from, statements);
        if (!retCode)
            retCode = bindValuesToStmt(values, stmt);
        if (!retCode)
            retCode = sqlite3_step(stmt);
        if (retCode == SQLITE_ROW) {
            sqlite3_column_value(stmt, 0, ref);
            retCode = SQLITE_OK;
        }
        if (stmt != nullptr)
            retCode = sqlite3_finalize(stmt);
        return retCode;
    }

    // Because I cannot get templates to work with strings
    void selectFrom(int& value,
                    const typedValuePairs& values,
                    const std::string& selector,
                    const std::string& from,
                    const std::string& statements = nullptr);
    void selectFrom(int& value,
                    const std::string& selector,
                    const std::string& from,
                    const std::string& statements = nullptr);
    void selectFrom(double& value,
                    const typedValuePairs& values,
                    const std::string& selector,
                    const std::string& from,
                    const std::string& statements = nullptr);
    void selectFrom(double& value,
                    const std::string& selector,
                    const std::string& from,
                    const std::string& statements = nullptr);
    void selectFrom(std::string& value,
                    const typedValuePairs& values,
                    const std::string& selector,
                    const std::string& from,
                    const std::string& statements = nullptr);
    void selectFrom(std::string& value,
                    const std::string& selector,
                    const std::string& from,
                    const std::string& statements = nullptr);

    // TODO: Below should probably be a template while the weird cases should be overloads

    // ints
    void selectFromId(int& result, const std::string& selector, const std::string& from, int id);
    void selectFromId(unsigned int& result,
                      const std::string& selector,
                      const std::string& from,
                      int id);
    void selectFromId(bool& result, const std::string& selector, const std::string& from, int id);

    // floats
    void selectFromId(double& result, const std::string& selector, const std::string& from, int id);
    void selectFromId(float& result, const std::string& selector, const std::string& from, int id);

    // strings
    void selectFromId(std::string& result,
                      const std::string& selector,
                      const std::string& from,
                      int id);

    std::vector<std::vector<std::string>> getRowsAsText(sqlite3_stmt*& stmt,
                                                        bool rowHeader = false);

    /*******************************************************************************
     * public
     ******************************************************************************/

   public:
    explicit DBHandler(std::string filePath);
    ~DBHandler();
    bool initialise();

	// get id from table returns either max or min id from table.
	// max = false -> min id, max = true -> max id
	enum ID_MINMAX { MIN_ID = false, MAX_ID = true };  // Uggly enum
	int getTableId(const std::string& table, ID_MINMAX = MAX_ID);

	// Receiver for log items from DBLogger
    void insertDataLogs(std::queue<LogItem>& logs);

    // Receivers for data coming in to the database from the website
    void receiveConfigs(const std::string& configsJSON);
    bool receiveWayPoints(const std::string& wayPointsJSON);

    // Config value reader
    template <typename T>
    void getConfigFrom(T& retVal,
                       const char* selector,
                       const char* from) {  // std::string version as well?
        selectFromId(retVal, std::string(selector), std::string(from), 1);
    }
    template <typename T>
    bool updateTableIdColumnValue(const char* table, int id, const char* colName, T newValue) {
        typedValuePairs values;
        addValue(values, colName, newValue);
        return updateTableRow(table, id, values);
    }
    template <typename T>
    bool updateTableColumnIdValue(const char* table, const char* column, int id, T value) {
        sqlite3_stmt* stmt = nullptr;
        std::string sql = "UPDATE " + std::string(table) + " SET " + std::string(column) +
                          " = :value WHERE ID = :id";
        if (!prepareStmtError(stmt, sql)) {
            bindParam(stmt, ":value", value);
            bindParam(stmt, ":id", id);
            if (stepAndFinalizeStmt(stmt) == SQLITE_DONE) {
                return true;
            }
        }
        // Logger::error("%s Error updating table %s using \"%s\"", __PRETTY_FUNCTION__, table,
        // sql.c_str()); Can not use logger in templates
        return false;
    }

    // Get dataLogs_* as JSON for sending to the website
    // supply onlyLatest to get only the ones with the highest id
    std::string getLogsAsJSON(int &afterId);

    // Empties all dataLog_* tables
    void clearLogs();

    // Waypoint value getter
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

    // Waypoints table as JSON
    std::string getWayPointsAsJSON();

    // Config tables as JSON
    std::string getConfigs();

    // Well ... umm
	void lock() { m_databaseLock.lock(); }
    void unlock() { m_databaseLock.unlock(); }

    // Generic string utility functions
    std::string prepend(const std::string& prefix, const std::string& str);
    std::vector<std::string> prepend(const std::string& prefix,
                                     const std::vector<std::string>& strings);
    std::string implode(const std::vector<std::string>& elements, const std::string& glue);
    std::vector<std::string> explode(const std::string& string, const char glue);
    int indexOfString(const std::vector<std::string>& haystack, const std::string& needle);
	bool deleteString(std::vector<std::string>& haystack, const std::string& needle);
};
