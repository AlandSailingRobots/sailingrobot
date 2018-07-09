#pragma once

#include <sqlite3.h>
#include <iostream>
#include <mutex>
#include <string>
#include <vector>

#include "../Libs/json/include/nlohmann/json.hpp"
#include "../Messages/CurrentSensorDataMsg.h"
#include "../Messages/WindStateMsg.h"
using JSON = nlohmann::json;

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
    sqlite3* m_DBHandle = NULL;

    // execute INSERT query and add new row into table
    bool DBTransaction(std::string SQLQuery);
    // bool DBTransaction(std::string SQLQuery, sqlite3 *db);

    // retrieve data from given table/tables, return value is a C 2D char array
    // rows and columns also return values (through a reference) about rows and columns in the
    // result set
    std::vector<std::string> retrieveFromTable(std::string SQLSelect, int& rows, int& columns);

    // adds a table row into the json object as a array if array flag is true,
    // otherwise it adds the table row as a json object
    // id field is not obligatory, can be left empty
    void getDataAsJson(std::string select,
                       std::string table,
                       std::string key,
                       std::string id,
                       JSON& js,
                       bool useArray);

    // gets the id column from a given table
    std::vector<std::string> getTableIds(std::string table);

    // gets all datatable names related to "ending" string
    // used to fetch all tables ending with _datalogs or _config
    std::vector<std::string> getTableNames(std::string like);

    // gets information(for instance: name/datatype) about all columns
    std::vector<std::string> getColumnInfo(std::string info, std::string table);

    // help function used in insertDataLog
    int insertLog(std::string table, std::string values);

    // own implementation of deprecated sqlite3_get_table()
    int getTable(const std::string& sql,
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

    // updates table with json string (data)
    bool updateTableJson(std::string table, std::string data);
    bool updateTableJsonObject(std::string table, JSON data);

    // updates table using values given
    bool updateTable(std::string table, std::string column, std::string value, std::string id);

    void clearTable(std::string table);

    void updateConfigs(std::string configs);
    bool updateWaypoints(std::string waypoints);

    // returns all logs in database as json; supply onlyLatest to get only the ones with the highest
    // id
    std::string getLogs(bool onlyLatest);

    void forceUnlock() { m_databaseLock.unlock(); }

    void clearLogs();

    // get id from table returns either max or min id from table.
    // max = false -> min id
    // max = true -> max id
    enum ID_MINMAX { MIN_ID = false, MAX_ID = true };
    int getTableId(const char *table, ID_MINMAX = MAX_ID);

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
    bool changeOneValue(std::string table, std::string newValue, std::string colName, int id);

    std::string getWaypoints();

    std::string getConfigs();

    // Private SQLite wrapper functions
    int checkRetCode(const int retCode) const;

    // For preparing
    int prepareStmtError(sqlite3_stmt *&stmt, std::string sql); // Ref here gave segfaults
    int prepareStmtSelectFromStatements(sqlite3_stmt *&stmt,
                                        const std::string &expressions,
                                        const std::string &tables,
                                        const std::string &statements = NULL);

/*    int prepareStmtSelectFromId(sqlite3_stmt** stmt,
                                const std::string& selector,
                                const std::string& from,
                                const int id);*/

    // For binding parameter values
    int paramNameIndex(sqlite3_stmt *&stmt, const char *name);
    int bindParam(sqlite3_stmt *&stmt, const char *name, const int &value);
    int bindParam(sqlite3_stmt *&stmt, const char *name, const double &value);
    int bindParam(sqlite3_stmt *&stmt, const char *name, const std::string &text);
    int bindStmtIntsDoublesStrings(
      sqlite3_stmt *&stmt,
      const std::vector<std::tuple<const char *, int>> &ints = {},
      const std::vector<std::tuple<const char *, double>> &doubles = {},
      const std::vector<std::tuple<const char *, std::string>> &strings = {});

    // Helpers
    int prepareAndBindSelectFromId(sqlite3_stmt *&stmt,
                                   const std::string& selector,
                                   const std::string& from,
                                   const int id);
    int stepAndFinalizeStmt(sqlite3_stmt *&stmt) const;

    // Retreiving data from SELECT queries


//    void sqlite3_column_value(sqlite3_stmt *stmt, int index, int &value);
//    void sqlite3_column_value(sqlite3_stmt *stmt, int index, double &value);
//    void sqlite3_column_value(sqlite3_stmt *stmt, int index, std::string &value);
	void sqlite3_column_value(sqlite3_stmt *&stmt, int index, int &value) {
		value = sqlite3_column_int(stmt, index);
	}
	void sqlite3_column_value(sqlite3_stmt *&stmt, int index, double &value) {
		value = sqlite3_column_double(stmt, index);
	}
	void sqlite3_column_value(sqlite3_stmt *&stmt, int index, std::string &value) {
		const char* strp = (char*)sqlite3_column_text(stmt, index);
		value = std::string(strp);
	}

/*    template <typename T>   // TODO wrap
    T selectFrom(const std::string& selector,
                 const std::string& from,
                 const std::string& statements = NULL,
                 const std::vector<std::tuple<const char*, int>>& ints = {},
                 const std::vector<std::tuple<const char*, double>>& doubles = {},
                 const std::vector<std::tuple<const char*, std::string>>& strings = {}) {
        sqlite3_stmt* stmt = NULL;
        // TODO: Error checking (retVals)
        prepareStmtSelectFromStatements(&stmt, selector, from, statements);
        bindStmtIntsDoublesStrings(&stmt, ints, doubles, strings);
        T retVal;
        sqlite3_column_value(stmt, 0, &retVal);
        sqlite3_finalize(stmt);
        return retVal;
    }*/

	template <typename T>
	int refSelectFromTemplate(T &ref,
	                          const std::string &selector,
	                          const std::string &from,
	                          const std::string &statements = NULL,
	                          const std::vector<std::tuple<const char *, int>> &ints = {},
	                          const std::vector<std::tuple<const char *, double>> &doubles = {},
	                          const std::vector<std::tuple<const char *, std::string>> &strings = {}) {
		int retCode = SQLITE_OK;
		sqlite3_stmt* stmt = NULL;

		retCode = prepareStmtSelectFromStatements(stmt, selector, from, statements);
		if (!retCode) retCode = bindStmtIntsDoublesStrings(stmt, ints, doubles, strings);
		if (!retCode) retCode = sqlite3_step(stmt);
		if (retCode == SQLITE_ROW) {
			sqlite3_column_value(stmt, 0, ref);
			retCode = SQLITE_OK;
		}
		if (stmt != NULL) retCode = sqlite3_finalize(stmt);
		return retCode;
	}

	// Because I cannot get templates to work with strings
	void selectFrom(
	  int& value,
	  const std::string& selector,
	  const std::string& from,
	  const std::string& statements = NULL,
	  const std::vector<std::tuple<const char*, int>>& ints = {},
	  const std::vector<std::tuple<const char*, double>>& doubles = {},
	  const std::vector<std::tuple<const char*, std::string>>& strings = {}) {
		refSelectFromTemplate(value, selector, from, statements, ints, doubles, strings);
	}
	void selectFrom(
	  double& value,
	  const std::string& selector,
	  const std::string& from,
	  const std::string& statements = NULL,
	  const std::vector<std::tuple<const char*, int>>& ints = {},
	  const std::vector<std::tuple<const char*, double>>& doubles = {},
	  const std::vector<std::tuple<const char*, std::string>>& strings = {}) {
		refSelectFromTemplate(value, selector, from, statements, ints, doubles, strings);
	}
	void selectFrom(
	  std::string& value,
	  const std::string& selector,
	  const std::string& from,
	  const std::string& statements = NULL,
	  const std::vector<std::tuple<const char*, int>>& ints = {},
	  const std::vector<std::tuple<const char*, double>>& doubles = {},
	  const std::vector<std::tuple<const char*, std::string>>& strings = {}) {
		refSelectFromTemplate(value, selector, from, statements, ints, doubles, strings);
	}

	// TODO: Below should probably be a template while the weird cases shuould be moved up into the overloads

	void selectFromId(bool& value, const std::string& selector, const std::string& from, int id) {
		int i;
		selectFrom(i, selector, from, "WHERE id = :id", {{":id", id}});
		value = ( i ? true : false );
	}
	void selectFromId(int& value, const std::string& selector, const std::string& from, int id) {
    	selectFrom(value, selector, from, "WHERE id = :id", {{":id", id}});
    }
	void selectFromId(unsigned int& value, const std::string& selector, const std::string& from, int id) {
		double d;
		selectFrom(d, selector, from, "WHERE id = :id", {{":id", id}});
		value = d; // TODO: double to unsigned int?
	}
	void selectFromId(float& value, const std::string& selector, const std::string& from, int id) {
		double d;
		selectFrom(d, selector, from, "WHERE id = :id", {{":id", id}});
		value = d;  // TODO: double to float?
	}
	void selectFromId(double& value, const std::string& selector, const std::string& from, int id) {
		selectFrom(value, selector, from, "WHERE id = :id", {{":id", id}});
	}
	void selectFromId(std::string& value, const std::string& selector, const std::string& from, int id) {
		selectFrom(value, selector, from, "WHERE id = :id", {{":id", id}});
	}

    template <typename T>
	void getConfigFrom(T &retVal, const char *selector, const char *from) {
		selectFromId(retVal, selector, from, 1);
	}

//	void configFrom(int& value, std::string& selector, std::string& from);
//	void configFrom(double &value, const char* selector, const char* from);
//	void configFrom(std::string& str, const char* selector, const char* from);

/*    int selectFromIdAsInt(const std::string& selector, const std::string& from, const int id);
    double getConfigsFrom(const std::string& selector, const std::string& from, const int id);
    std::string selectFromIdAsString(const std::string& selector,
                                     const std::string& from,
                                     const int id);*/

    std::vector<std::vector<std::string>> rowsAsText(sqlite3_stmt *&stmt);





    // FOR REFACTORING:



};
