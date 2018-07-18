#pragma once

// TODO really move stuff from public to private

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
    int m_latestDataLogId = 0;
    std::string m_currentWaypointId = "";
    std::string m_filePath;
    static std::mutex m_databaseLock;
    sqlite3* m_DBHandle = nullptr;

    // execute INSERT query and add new row into table
    bool DBTransaction(const std::string &SQLQuery);
    // bool DBTransaction(std::string SQLQuery, sqlite3 *db);

    // retrieve data from given table/tables, return value is a C 2D char array
    // rows and columns also return values (through a reference) about rows and columns in the
    // result set
    std::vector<std::string> retrieveFromTable(const std::string &SQLSelect, int &rows, int &columns);

    // adds a table row into the json object as a array if array flag is true,
    // otherwise it adds the table row as a json object
    // id field is not obligatory, can be left empty
/*    void getDataAsJson(std::string select,
                       std::string table,
                       std::string key,
                       std::string id,
                       JSON& js,
                       bool useArray);*/

    // gets the id column from a given table
    /*std::vector<std::string> getTableIds(std::string table);*/

    // gets all datatable names related to "ending" string
    // used to fetch all tables ending with _datalogs or _config
    std::vector<std::string> getTableNames(std::string like);

    // gets information(for instance: name/datatype) about all columns
    std::vector<std::string> getColumnInfo(std::string &info, std::string &table);

    // help function used in insertDataLog
    /*int insertLog(std::string table, std::string values);*/

    // own implementation of deprecated sqlite3_get_table()
    int getTable(const std::string& sql,
                 std::vector<std::string>& results,
                 int& rows,
                 int& columns);

    sqlite3* DBConnect();
    void DBDisconnect();
    void DBClose();

   public:
	explicit DBHandler(std::string filePath);
    ~DBHandler();

    bool initialise();

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

    /*int getRows(std::string table);*/

    void insertDataLogs(std::vector<LogItem>& logs);

    // updates table with json string (data)
    bool updateTableJson(const std::string &table, const std::string &data);
    /* bool updateTableJsonObject(std::string table, JSON data); */

    // updates table using values given
    //  bool updateTableColumnIdValue(std::string table, std::string column, int id, T value);
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

    void clearTable(const std::string &table);

    bool JSONAsTables(const std::string &string, textTables &tables);

    void updateConfigs(const std::string &configsJSON);
    bool receiveWayPoints(const std::string &wayPointsJSON);

    DBHandler::textTables getTablesAsText(const std::string &like, const std::string &statement = "");
    std::string getTablesAsJSON(const std::string &like, const std::string &statement = "");
    // returns all logs in database as json; supply onlyLatest to get only the ones with the highest
    // id
    std::string getLogsAsJSON(bool onlyLatest);

    void forceUnlock() { m_databaseLock.unlock(); }

    void clearLogs();

    // get id from table returns either max or min id from table.
    // max = false -> min id
    // max = true -> max id
    enum ID_MINMAX { MIN_ID = false, MAX_ID = true };
    int getTableId(const std::string &table, ID_MINMAX = MAX_ID);

    // not implemented
    // void deleteRow(std::string table, std::string id);

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

    std::string getWayPointsAsJSON();

    std::string getConfigs();

    // Private SQLite wrapper functions
    int checkRetCode(int retCode) const;

    // For preparing
    int prepareStmtError(sqlite3_stmt *&stmt, const std::string &sql);  // Ref here gave segfaults
    int prepareStmtSelectFromStatements(sqlite3_stmt*& stmt,
                                        const std::string& expressions,
                                        const std::string& tables,
                                        const std::string& statements = nullptr);
    int paramNameIndex(sqlite3_stmt *&stmt, std::string name);

    int bindValuesToStmt(const typedValuePairs& values, sqlite3_stmt*& stmt);
    void addValue(typedValuePairs &values, const std::string &name, int value);
    void addValue(typedValuePairs &values, const std::string &name, double value);
    void addValue(typedValuePairs &values, const std::string &name, std::string &string);
    std::vector<std::string> valueNames(const typedValuePairs& values);
    int bindParam(sqlite3_stmt *&stmt, const std::string &name, int value);
    int bindParam(sqlite3_stmt *&stmt, const std::string &name, double value);
    int bindParam(sqlite3_stmt *&stmt, const std::string &name, const std::string &text);
    int bindStmtIntsDoublesStrings(
      sqlite3_stmt *&stmt,
      const std::vector<std::pair<const std::string, int>> &ints = {},
      const std::vector<std::pair<const std::string, double>> &doubles = {},
      const std::vector<std::pair<const std::string, std::string>> &strings = {});

    // INSERT
    int prepareStmtInsertError(sqlite3_stmt*& stmt,
                               const std::string& table,
                               std::vector<std::string>& columns);
    int prepareStmtInsertError(sqlite3_stmt *&stmt,
                               const std::string &table,
                               const typedValuePairs &values);
    bool insertTableRow(const std::string &tableName, const typedValuePairs &values);
    int insertTableRowsErrors(const std::string &tableName, const tableRows &rows);

    bool transactionalReplaceTable(const std::string &tableName, const tableRows &rows);
    bool transactionalReplaceTable(const std::string &tableName, const textTableRows &rows);
    bool transactionalReplaceTable(const textTable &table);
    bool replaceTables(const textTables &tables);

    // UPDATE
    int prepareStmtUpdateError(sqlite3_stmt *&stmt,
                               const std::string &table,
                               const int id,
                               const std::vector<std::string> &columns);
    int prepareStmtUpdateError(sqlite3_stmt *&stmt,
                               const std::string &table,
                               const int id,
                               const typedValuePairs &values);

    bool updateTableRow(const std::string &table, int id, const typedValuePairs &values);
    template <typename T>
    bool updateTableIdColumnValue(const char* table, int id, const char* colName, T newValue) {
        typedValuePairs values;
        addValue(values, colName, newValue);
        return updateTableRow(table, id, values);
    }

    void valuesFromTextRows(tableRows &values, const textTableRows &textRows, const ColumnTypes &types);

    // TODO: A select of multiple values into typedValuePairs struct would be nice
    // No... a row and named column name index!

    /* Not in use
    int prepareAndBindSelectFromId(sqlite3_stmt*& stmt,
                                   const std::string& selector,
                                   const std::string& from,
                                   const int id); */
    int stepAndFinalizeStmt(sqlite3_stmt*& stmt) const;

    // Retreiving data from SELECT queries
    // TODO: move out from headers

    //    void sqlite3_column_value(sqlite3_stmt *stmt, int index, int &value);
    //    void sqlite3_column_value(sqlite3_stmt *stmt, int index, double &value);
    //    void sqlite3_column_value(sqlite3_stmt *stmt, int index, std::string &value);
    void sqlite3_column_value(sqlite3_stmt*& stmt, int index, int& value) {
        value = sqlite3_column_int(stmt, index);
    }
    void sqlite3_column_value(sqlite3_stmt*& stmt, int index, bool& value) {
        value = (sqlite3_column_int(stmt, index) != 0);
    }
    void sqlite3_column_value(sqlite3_stmt*& stmt, int index, double& value) {
        value = sqlite3_column_double(stmt, index);
    }
    void sqlite3_column_value(sqlite3_stmt*& stmt, int index, std::string& value) {
        if (sqlite3_column_type(stmt, index) == SQLITE_NULL) {
            value = std::string();
        } else {
            const char* strp = (char*)sqlite3_column_text(stmt, index);
            value = std::string(strp);
        }
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
            bindValuesToStmt(&stmt, ints, doubles, strings);
            T retVal;
            sqlite3_column_value(stmt, 0, &retVal);
            sqlite3_finalize(stmt);
            return retVal;
        }*/

    /*template <typename T>
    int refSelectFromTemplate(
      T &ref,
      const std::string &selector,
      const std::string &from,
      const std::string &statements = nullptr,
      const std::vector<std::pair<const std::string &, int>> &ints = {},
      const std::vector<std::pair<const std::string &, double>> &doubles = {},
      const std::vector<std::pair<const std::string &, std::string>> &strings = {}) {
        int retCode;
        sqlite3_stmt* stmt = nullptr;

        retCode = prepareStmtSelectFromStatements(stmt, selector, from, statements);
        if (!retCode)
            retCode = bindStmtIntsDoublesStrings(stmt, ints, doubles, strings);
        if (!retCode)
            retCode = sqlite3_step(stmt);
        if (retCode == SQLITE_ROW) {
            sqlite3_column_value(stmt, 0, ref);
            retCode = SQLITE_OK;
        }
        if (stmt != nullptr)
            retCode = sqlite3_finalize(stmt);
        return retCode;
    }*/

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
    void selectFrom(int &value,
                    const typedValuePairs& values,
                    const std::string &selector,
                    const std::string &from,
                    const std::string &statements = nullptr) {
	    refSelectFromTemplate(value, values, selector, from, statements);
    }
	void selectFrom(int &value,
	                const std::string &selector,
	                const std::string &from,
	                const std::string &statements = nullptr) {
		refSelectFromTemplate(value, {{},{},{}}, selector, from, statements);
	}
	void selectFrom(double &value,
	                const typedValuePairs& values,
	                const std::string &selector,
	                const std::string &from,
	                const std::string &statements = nullptr) {
		refSelectFromTemplate(value, values, selector, from, statements);
	}
	void selectFrom(double &value,
	                const std::string &selector,
	                const std::string &from,
	                const std::string &statements = nullptr) {
		refSelectFromTemplate(value, {{},{},{}}, selector, from, statements);
	}
	void selectFrom(std::string &value,
	                const typedValuePairs& values,
	                const std::string &selector,
	                const std::string &from,
	                const std::string &statements = nullptr) {
		refSelectFromTemplate(value, values, selector, from, statements);
	}
	void selectFrom(std::string &value,
	                const std::string &selector,
	                const std::string &from,
	                const std::string &statements = nullptr) {
		refSelectFromTemplate(value, {{},{},{}}, selector, from, statements);
	}
    /*
    void selectFrom(int &value,
                    const std::string &selector,
                    const std::string &from,
                    const std::string &statements = nullptr,
                    const std::vector<std::pair<const std::string &, int>> &ints = {},
                    const std::vector<std::pair<const std::string &, double>> &doubles = {},
                    const std::vector<std::pair<const std::string &, std::string>> &strings = {}) {
        refSelectFromTemplate(value, selector, from, statements, ints, doubles, strings);
    }
    void selectFrom(double &value,
                    const std::string &selector,
                    const std::string &from,
                    const std::string &statements = nullptr,
                    const std::vector<std::pair<const std::string &, int>> &ints = {},
                    const std::vector<std::pair<const std::string &, double>> &doubles = {},
                    const std::vector<std::pair<const std::string &, std::string>> &strings = {}) {
        refSelectFromTemplate(value, selector, from, statements, ints, doubles, strings);
    }
    void selectFrom(std::string &value,
                    const std::string &selector,
                    const std::string &from,
                    const std::string &statements = nullptr,
                    const std::vector<std::pair<const std::string &, int>> &ints = {},
                    const std::vector<std::pair<const std::string &, double>> &doubles = {},
                    const std::vector<std::pair<const std::string &, std::string>> &strings = {}) {
        refSelectFromTemplate(value, selector, from, statements, ints, doubles, strings);
    }
*/

    // TODO: Below should probably be a template while the weird cases should be moved up into the
    // overloads

    // ints
    void selectFromId(int& result, const std::string& selector, const std::string& from, int id) {
	    selectFrom(result, {{std::make_pair("id", id)},{},{}}, selector, from, "WHERE id = :id");
    }
	void selectFromId(unsigned int& result, const std::string& selector, const std::string& from, int id) {
		int i;
		selectFromId(i, selector, from, id);
		result = (unsigned int) i;
	}
	void selectFromId(bool& result, const std::string& selector, const std::string& from, int id) {
    	int i;
		selectFromId(i, selector, from, id);
		result = (i != 0);
	}

	// floats
	void selectFromId(double& result, const std::string& selector, const std::string& from, int id) {
		selectFrom(result, {{std::make_pair("id", id)},{},{}}, selector, from, "WHERE id = :id");

	}
    void selectFromId(float& result, const std::string& selector, const std::string& from, int id) {
        double d;
	    selectFromId(d, selector, from, id);
        result = (float) d;
    }

    // strings
    void selectFromId(std::string& result,
                      const std::string& selector,
                      const std::string& from,
                      int id) {
	    selectFrom(result, {{std::make_pair("id", id)},{},{}}, selector, from, "WHERE id = :id");
    }

    template <typename T>
    void getConfigFrom(T& retVal, const char* selector, const char* from) { // REDO
        selectFromId(retVal, std::string(selector), std::string(from), 1);
    }

    std::vector<std::vector<std::string>> getRowsAsText(sqlite3_stmt*& stmt,
                                                        bool rowHeader = false);
	int sqliteTypeFromString(std::string const &typestr);
    ColumnTypes getTableColumnTypes(const std::string &tableName);
	int columnType(const std::string &name, ColumnTypes &types);

    // Generic string utility functions
    std::string prependString(const std::string &prefix, const std::string &str);
    std::vector<std::string> prependStrings(const std::string &prefix, const std::vector<std::string> &strings);
    std::string joinStrings(const std::vector<std::string> &elements, const std::string &glue);
    std::vector<std::string> splitStrings(const std::string& string, const char glue);
    int indexOfStringInStrings(const std::vector<std::string> &haystack, const std::string &needle);
};
