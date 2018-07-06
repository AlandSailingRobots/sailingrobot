#include <cstdio>
#include <cstdlib>
#include <iomanip>
#include <sstream>
#include <string>
#include <thread>

#include "../SystemServices/Logger.h"
#include "../SystemServices/Timer.h"
#include "../SystemServices/Wrapper.h"
#include "DBHandler.h"

// TODO:
// SQLITE_TRANSIENT tells SQLite to copy your string. Use this when your string('s buffer) is going
// to go away before the query is executed. SQLITE_STATIC tells SQLite that you promise that the
// pointer you pass to the string will be valid until after the query is executed. Use this when
// your buffer is, um, static, or at least has dynamic scope that exceeds that of the binding.
// https://stackoverflow.com/questions/1711631/improve-insert-per-second-performance-of-sqlite

// TODO: reusable statements?

std::mutex DBHandler::m_databaseLock;

DBHandler::DBHandler(std::string filePath) : m_filePath(std::move(filePath)) {
    m_latestDataLogId = 0;
}

DBHandler::~DBHandler() {
    DBClose();
    m_databaseLock.unlock();
}

bool DBHandler::initialise() {
    sqlite3* connection = DBConnect();  // Basically just try to open the DB
    DBDisconnect();
    return connection;  // NULL would mean we got no connection
}

/**
 * Checks and logs possible SQLire error codes
 * @param retCode
 * @return retCode
 */
int DBHandler::checkRetCode(const int retCode) const {
    if (not((retCode == SQLITE_OK) || (retCode == SQLITE_DONE) || (retCode == SQLITE_ROW))) {
        Logger::error("SQLite result code: %s (%d)", sqlite3_errstr(retCode), retCode);
    }
    return retCode;
}

int DBHandler::stepAndFinalizeStmt(sqlite3_stmt *stmt) const {
    int retCode = checkRetCode(sqlite3_step(stmt));
    checkRetCode(sqlite3_finalize(stmt));  // Only for logging possible error
    return retCode;
}


// TODO: Rewrite old
bool DBHandler::updateTableJson(std::string table, std::string data) {
    std::vector<std::string> columns = getColumnInfo("name", table);

    if (columns.size() <= 0) {
        Logger::error("%s Error: no such table %s", __PRETTY_FUNCTION__, table.c_str());
        return false;
    }

    JSON js = JSON::parse(data);

    std::stringstream ss;

    // start at i = 1 to skip the id
    ss << "SET ";
    int fixedSize = js.size();  // Size would sometimes change, added this variable
    for (auto i = 1; i < fixedSize; i++) {
        if (fixedSize > 1) {
            ss << columns.at(i) << " = " << js[columns.at(i)]
               << ",";  // This crashes if the local database has fewer fields than the web database
                        // (field out of range)
        }
    }

    std::string values = ss.str();
    values = values.substr(0, values.size() - 1);

    std::string id = js["id"];

    if (not DBTransaction("UPDATE " + table + " " + values + " WHERE ID = " + id + ";")) {
        Logger::error("%s Error: ", __PRETTY_FUNCTION__);
        return false;
    }
    return true;
}

/*bool DBHandler::updateTableJsonObject(std::string table, JSON data) {
    // m_logger.info(" updateTableJson:\n"+data);
    std::vector<std::string> columns = getColumnInfo("name", table);

    if (columns.size() <= 0) {
        Logger::error("%s Error: no such table %s", __PRETTY_FUNCTION__, table.c_str());
        return false;
    }

    // JSON json = data;

    std::stringstream ss;

    // start at i = 1 to skip the id
    ss << "SET ";
    int fixedSize = data.size();  // Size would sometimes change, added this variable
    for (auto i = 1; i < fixedSize; i++) {
        if (fixedSize > 1) {
            ss << columns.at(i) << " = " << data[columns.at(i)]
               << ",";  // This crashes if the local database has fewer fields than the web database
                        // (field out of range)
        }
    }

    std::string values = ss.str();
    values = values.substr(0, values.size() - 1);

    std::string id = data["id"];

    if (not DBTransaction("UPDATE " + table + " " + values + " WHERE ID = " + id + ";")) {
        Logger::error("%s Error: ", __PRETTY_FUNCTION__);
        return false;
    }
    return true;
}*/

// TODO: Rewrite old
bool DBHandler::updateTable(std::string table,
                            std::string column,
                            std::string value,
                            std::string id) {
    if (not DBTransaction("UPDATE " + table + " SET " + column + " = " + value +
                          " WHERE ID = " + id + ";")) {
        Logger::error("%s Error updating table", __PRETTY_FUNCTION__);
        return false;
    }
    return true;
}

// TODO: Rewrite old
void DBHandler::updateConfigs(std::string configs) {
    JSON js = JSON::parse(configs);
    if (js.empty()) {
        Logger::error("%s No JSON in \"%s\"", __PRETTY_FUNCTION__, configs);
    }
    std::vector<std::string> tables;

    for (auto i : js.items()) {
        tables.push_back(i.key());  // For each table key
    }

    // tables = sailing_config config_buffer etc

    for (auto table : tables) {  // for each table in there
        if (js[table] != NULL) {
            updateTableJson(table, js[table].dump());  // eg updatetablejson("sailing_config",
                                                       // configs['sailing_config'] as json)
        }
    }
}

// TODO: Rewrite old
bool DBHandler::updateWaypoints(std::string waypoints) {
    JSON js = JSON::parse(waypoints);
    if (js.empty()) {
        Logger::error("%s No JSON in \"%s\"", __PRETTY_FUNCTION__, waypoints);
    }
    std::string DBPrinter = "";
    std::string tempValue = "";
    int valuesLimit = 11;  //"Dirty" fix for limiting the amount of values requested from server
                           // waypoint entries (amount of fields n = valuesLimit + 1)
    int limitCounter;

    if (not DBTransaction("DELETE FROM current_Mission;")) {
        Logger::error("%s, Error: failed to delete waypoints", __PRETTY_FUNCTION__);
    }

    for (auto i : js.items()) {
        // m_logger.info(i.value().dump());

        for (auto y : i.value().items()) {
            limitCounter = valuesLimit;
            DBPrinter =
                "INSERT INTO current_Mission "
                "(declination,harvested,id,id_mission,is_checkpoint,latitude,longitude,name,radius,"
                "rankInMission,stay_time) VALUES (";

            for (auto z : y.value().items()) {
                // Each individual value
                tempValue = z.value().dump();
                tempValue = tempValue.substr(1, tempValue.size() - 2);
                if (tempValue == "") {
                    tempValue = "NULL";
                }
                if (limitCounter > 0) {
                    limitCounter--;
                    DBPrinter = DBPrinter + tempValue + ",";
                }
            }

            // if (DBPrinter.size () > 0)  DBPrinter.resize (DBPrinter.size () - 1);
            // DBPrinter = DBPrinter + "0);";
            DBPrinter = DBPrinter.substr(0, DBPrinter.size() - 1) + ");";
            std::cout << DBPrinter << "\n";
            if (not DBTransaction(DBPrinter)) {
                Logger::error("%s, Error: failed to add waypoints", __PRETTY_FUNCTION__);
                return false;
            }
        }
    }

    // Make sure waypoints before the current waypoint are harvested
    if (!m_currentWaypointId.empty()) {
        std::string updateHarvested = "UPDATE current_Mission SET harvested = 1 WHERE id < ";
        updateHarvested += m_currentWaypointId + ";";

        if (not DBTransaction(updateHarvested)) {
            Logger::error("%s, Error: failed to harvest waypoints", __PRETTY_FUNCTION__);
            return false;
        }
    }
    return true;
}

// TODO: Rewrite old
void DBHandler::clearTable(std::string table) {
    // If no table to delete, doesn't matter
    DBTransaction("DELETE FROM " + table + ";");
}

// This is basically a COUNT not using any values
// TODO: Rewrite old, possibly kill
int DBHandler::getRows(std::string table) {
    int columns, rows;
    try {
        retrieveFromTable("SELECT * FROM " + table + ";", rows, columns);
    } catch (const char* error) {
        return 0;
    }
    return rows;
}

// TODO: review all usages of selectFrom function in other parts of the project

// TODO: Rewrite old
std::string DBHandler::getConfigs() {
    JSON js;

    // Fetch all table names ending with "_config"
    std::vector<std::string> configTables =
        getTableNames("config_%");  // NOTE : Marc : Modify this point

    // Query config tables and select all from config tables with id "1"
    // This json structure does not use arrays
    for (auto table : configTables) {
        getDataAsJson("*", table, table, "1", js, false);
    }

    return js.dump();
}

// TODO: Rewrite old
void DBHandler::clearLogs() {
    std::vector<std::string> datalogTables = getTableNames("dataLogs_%");

    for (auto table : datalogTables) {
        clearTable(table);
    }
}

// TODO: Rewrite old
std::string DBHandler::getWaypoints() {  // NOTE : Marc : change this otherwise it doesn't work
    int rows = 0;
    JSON js;
    std::string wp = "waypoint_";

    rows = getRows("current_Mission");
    // std::cout << "rows current mission " << rows << std::endl;
    if (rows > 0) {
        for (auto i = 1; i <= rows; ++i) {
            // getDataAsJson("id,is_checkpoint,latitude,longitude,declination,radius,stay_time",
            // "current_Mission", wp + std::to_string(i), std::to_string(i),json, true);
            getDataAsJson("id,is_checkpoint,latitude,longitude,declination,radius,stay_time",
                          "current_Mission", wp + std::to_string(i), std::to_string(i), js, true);
        }
        return js.dump();
    } else {
        Logger::warning("No waypoints in database");
        return "";
    }
}

// get id from table returns either max or min id from table.
// max = false -> min id
// max = true -> max id
// TODO: Rewrite to use generic
int DBHandler::getTableId(std::string table, ID_MINMAX minmax) {
	sqlite3_stmt** stmt = NULL;
	int id = 0;
	try {
	    prepareStmtSelectFromStatements(stmt, (minmax == MAX_ID ? "MAX(id)" : "MIN(id)"), table);
		sqlite3_column_value(*stmt, 0, id);
		sqlite3_finalize(*stmt);
    } catch (const char* error) {
        Logger::error("%s Error when determining min/max id from %s", __PRETTY_FUNCTION__, table);
    }
    return id;
}

/******************************************************************************
 * private helpers
 *****************************************************************************/

/*******************************************************************************
 * Database stuff
 ******************************************************************************/

/**
 * Database handle getter
 * @return DB handle
 */
sqlite3* DBHandler::DBConnect() {
    if (!m_DBHandle) {
        m_databaseLock.lock();
        Logger::info("%s Opening database %s", __PRETTY_FUNCTION__, m_filePath.c_str());
        int resultcode = 0;

        // check if file exists
        FILE* db_file = fopen(m_filePath.c_str(), "r");
        if (db_file == NULL) {
            Logger::error("%s %s not found", __PRETTY_FUNCTION__, m_filePath.c_str());
            m_databaseLock.unlock();
            return NULL;
        }
        fclose(db_file);

        do {
            resultcode = sqlite3_open(m_filePath.c_str(), &m_DBHandle);
        } while (resultcode == SQLITE_BUSY);

        if (resultcode) {
            Logger::error("%s Failed to open the database Error %s", __PRETTY_FUNCTION__,
                          sqlite3_errmsg(m_DBHandle));
            m_databaseLock.unlock();
            return 0;
        }

        // set a 10 millisecond timeout
        sqlite3_busy_timeout(m_DBHandle, 10);
        m_databaseLock.unlock();
    }
    return m_DBHandle;
}

/**
 * Does nothing at the moment
 */
void DBHandler::DBDisconnect() {
    // Logger::info("%s Database NOP disconnect", __PRETTY_FUNCTION__);
}

/**
 * Close the DB file if open
 */
void DBHandler::DBClose() {
    if (m_DBHandle != NULL) {
        Logger::info("%s closing Database", __PRETTY_FUNCTION__);
        sqlite3_close(m_DBHandle);
        m_DBHandle = NULL;
        // Ensure it closes properly (by sleeping a millisecond)?
        // std::this_thread::sleep_for(std::chrono::milliseconds(1));
        m_databaseLock.unlock();
    } else {
        m_databaseLock.unlock();
        throw "DBHandler::DBClose() : connection is already null";
    }
}

/*******************************************************************************
 * PDO statement preparations
 ******************************************************************************/

// internal!

/**
 *
 * @param db
 * @param sql
 * @param stmt
 * @return
 */
int DBHandler::prepareStmtError(sqlite3_stmt** stmt, const std::string& sql) {
	sqlite3* db = DBConnect();
	int resultCode = checkRetCode(sqlite3_prepare_v2(db, sql.c_str(), (int)sql.size(), stmt, NULL));
	if (resultCode != SQLITE_OK) {
		Logger::error("%s SQL statement prepare error: %s on \"%s\"", __PRETTY_FUNCTION__,
		              sqlite3_errstr(resultCode), sql.c_str());
	}
	return resultCode;
}

/* SELECT expressions
 * FROM tables
 *	  [WHERE conditions]
 *	  [ORDER BY expression [ ASC | DESC ]]
 * LIMIT number_rows [ OFFSET offset_value ];
 */

/**
 * Prepares an SQLite statement out of SQL parts as strings
 * @param stmt 			Statement pointer
 * @param expressions	SELECT expressions
 * @param tables 		FROM tables
 * @return				SQLite return code
 */
int DBHandler::prepareStmtSelectFromStatements(sqlite3_stmt** stmt,
                                               const std::string& expressions,
                                               const std::string& tables,
                                               const std::string& statements) {
	std::string sql = "SELECT " + tables + " FROM " + expressions;
	if (statements.length()) {
		sql += " " + statements;
	}
	sql += ";";  // Not really needed but neat
	return prepareStmtError(stmt, sql);
}

int DBHandler::prepareAndBindSelectFromId(sqlite3_stmt** stmt,
                                       const std::string& selector,
                                       const std::string& from,
                                       const int id) {
	int retCode;
	retCode = prepareStmtSelectFromStatements(stmt, selector, from, "WHERE id = :id");
	if (retCode == SQLITE_OK) {
		retCode = bindStmtIntsDoublesStrings(stmt, {{":id", id}});
	};
	return retCode;
}

/*******************************************************************************
 * PDO Parameter binding
 ******************************************************************************/

/**
 * Binds an integer to a statement
 * @param stmt		SQLite statement
 * @param name		Parameter name
 * @param value		Value
 * @return
 */
int DBHandler::bindParam(sqlite3_stmt* stmt, const char* name, const int& value) {
    int paramIndex = paramNameIndex(stmt, name);
    if (!paramIndex) {
        return SQLITE_MISUSE;
    }
    return checkRetCode(sqlite3_bind_int(stmt, paramIndex, value));
}

/**
 * Binds a double to a statement
 * @param stmt		SQLite statement
 * @param name		Parameter name
 * @param value		Value
 * @return
 */
int DBHandler::bindParam(sqlite3_stmt* stmt, const char* name, const double& value) {
    int paramIndex = paramNameIndex(stmt, name);
    if (!paramIndex) {
        return SQLITE_MISUSE;
    }
    return checkRetCode(sqlite3_bind_double(stmt, paramIndex, value));
}

/**
 * Returns the numerical index of a named parameter
 * @param stmt
 * @param name
 * @return
 */
int DBHandler::paramNameIndex(sqlite3_stmt* stmt, const char* name) {
	int paramIndex = sqlite3_bind_parameter_index(stmt, name);
	if (!paramIndex)
		Logger::error("SQLite null parameter index on \"%s\"!", name);
	return paramIndex;
}

/**
 * Binds a string to a statement
 * @param stmt		SQLite statement
 * @param name		Parameter name
 * @param value		Value
 * @return
 */
int DBHandler::bindParam(sqlite3_stmt* stmt, const char* name, const std::string& text) {
    int paramIndex = paramNameIndex(stmt, name);
    if (!paramIndex) {
        return SQLITE_MISUSE;
    }
    return checkRetCode(
        sqlite3_bind_text(stmt, paramIndex, text.c_str(), text.size(), SQLITE_TRANSIENT));
}

/**
 * Prepares an SQLite statement and binds parameters
 * @param stmt 		Statement pointer
 * @param ints 		Name-value paired integer params
 * @param doubles 	Name-value paired double params
 * @param strings 	Name-value paired string params
 * @return 			SQLite error code ...
 */
int DBHandler::bindStmtIntsDoublesStrings(
    sqlite3_stmt** stmt,
    const std::vector<std::tuple<const char*, int>>& ints,
    const std::vector<std::tuple<const char*, double>>& doubles,
    const std::vector<std::tuple<const char*, std::string>>& strings) {
    int firstErrorCode = SQLITE_OK;
    int retCode;  // Uggly but works

    for (auto tup : ints) {
        firstErrorCode = bindParam(*stmt, std::get<0>(tup), std::get<1>(tup));
    }
    for (auto tup : doubles) {
        retCode = bindParam(*stmt, std::get<0>(tup), std::get<1>(tup));
        if (firstErrorCode == SQLITE_OK)
            firstErrorCode = retCode;
    }
    for (auto tup : strings) {
        retCode = bindParam(*stmt, std::get<0>(tup), std::get<1>(tup));
        if (firstErrorCode == SQLITE_OK)
            firstErrorCode = retCode;
    }
    return firstErrorCode;
}

void sqlite3_column_value(sqlite3_stmt* stmt, int index, int& value) {
	value = sqlite3_column_int(stmt, index);
}
void sqlite3_column_value(sqlite3_stmt* stmt, int index, double& value) {
	value = sqlite3_column_double(stmt, index);
}
void sqlite3_column_value(sqlite3_stmt* stmt, int index, std::string& value) {
	const char* strp = (char*)sqlite3_column_text(stmt, 0);
	value = std::string(strp);
}

// int getConfigFrom("loop_time","config_blah");
// double getConfigFrom("loop_time","config_blah");


/*int DBHandler::selectFromAsInt(const std::string& selector, const std::string& from) {
	sqlite3_stmt* stmt = NULL;

	stmtSelectFrom(&stmt, selector, from, id);
	int retVal = sqlite3_column_int(stmt, 0);
	sqlite3_finalize(stmt);
	return retVal;
}*/

/*std::string DBHandler::getConfigFromAsText(const char* selector, const char* from) {
	std::string retVal;
	refSelectFrom(retVal, std::string(selector), std::string(from), "WHERE id = :id", {{":id", 1}});
	return retVal;
}*/

/*
int DBHandler::selectFromIdAsInt(const std::string& selector,
                                 const std::string& from,
                                 const int id) {
	sqlite3_stmt* stmt = NULL;
	prepareAndBindSelectFromId(&stmt, selector, from, id);
	int retVal = sqlite3_column_int(stmt, 0);
	sqlite3_finalize(stmt);
	return retVal;
}

double DBHandler::getConfigsFrom(const std::string& selector,
                                       const std::string& from,
                                       const int id) {
	sqlite3_stmt* stmt = NULL;
	prepareAndBindSelectFromId(&stmt, selector, from, id);
	double retVal = sqlite3_column_double(stmt, 0);
	sqlite3_finalize(stmt);
	return retVal;
}

std::string DBHandler::selectFromIdAsString(const std::string& selector,
                                        const std::string& from,
                                        const int id) {
	sqlite3_stmt* stmt = NULL;
	std::string text;

	prepareAndBindSelectFromId(&stmt, selector, from, id);
	const char* strp = (char*)sqlite3_column_text(stmt, 0);
	sqlite3_finalize(stmt);
	return std::string(strp);
}
*/
// TODO: some push could be emplace?
// TODO: BUSY loop
// TODO: refs instead of copies - rewrite using new
std::vector<std::vector<std::string>> DBHandler::rowsAsText(sqlite3_stmt** stmt) {
    std::vector<std::vector<std::string>> rows;
    int retCode = sqlite3_step(*stmt);
    if (not((retCode == SQLITE_ROW) || (retCode == SQLITE_DONE))) {
        Logger::error("%s SQLite error: %s (%d)", __PRETTY_FUNCTION__,
                      sqlite3_errstr(retCode));
    } else {
        int columns = sqlite3_column_count(*stmt);
        std::vector<std::string> row;
        while (retCode == SQLITE_ROW) {
            for (int i = 0; i < columns; i++) {
                char* strp = (char*)sqlite3_column_text(*stmt, i);
                std::string retStr = std::string(strp);
                // TODO: Maybe disable this non-UTF-8 content check here? Enable again on JSON
                // crashes.
                for (char c : retStr) {
                    if (!std::isprint(c)) {
                        Logger::error("%s invalid UTF-8 character found in SQLITE DB TEXT: \"%s\"",
                                      __PRETTY_FUNCTION__, retStr.c_str());
                        retStr = std::string();
                    }
                }
                row.push_back(retStr);
            }
            rows.push_back(row);
            retCode = sqlite3_step(*stmt);
            checkRetCode(retCode);  // TODO inline above
        }
    }
    checkRetCode(sqlite3_finalize(*stmt));
    return rows;
}

/*******************************************************************************
 * INSERT and UPDATE ops
 ******************************************************************************/

// TODO: REMOVE COMMENTED OUT OLD CODE WHEN TESTED
// TODO: another rewrite
bool DBHandler::changeOneValue(std::string table,
                               std::string newValue,
                               std::string colName,
                               int id) {
	sqlite3_stmt* stmt = NULL;
	std::string sql = "UPDATE " + table + " SET " + colName + " = :value";
	if (id)
		sql += " WHERE id = :id";

	if (!prepareStmtError(&stmt, sql)) {
		bindParam(stmt, ":value", newValue);
		if (id) {
			bindParam(stmt, ":id", id);
		}
		if (!stepAndFinalizeStmt(stmt)) {
			Logger::error("%s Error on \"%s\"", __PRETTY_FUNCTION__, sql.c_str());
			return false;
		}
	}

	/*
		if (not DBTransaction("UPDATE " + table + " SET " + colName + " = " + newValue +
							  " WHERE id = " + id + ";")) {
			Logger::error("Error %s", __PRETTY_FUNCTION__);
			return false;
		}
	*/
	return true;
}

/**
 * Inserts all current values in the DB. Uses overloaded bindParam instead of
 * generic bining so you do not have to worry about types
 * @param logs Object contaning data to be logged
 */
void DBHandler::insertDataLogs(std::vector<LogItem>& logs) {
	int actuatorFeedbackId = -1;
	int compassModelId = -1;
	int courseCalculationId = -1;
	int currentMissionId = -1;
	int currentSensorsId = -1;
	int gpsId = -1;
	int marineSensorsId = -1;
	int vesselStateId = -1;
	int windStateId = -1;
	int windsensorId = -1;

	int logNumber = 0;
	std::string tableId;

	if (logs.empty()) {
		Logger::error("%s Nothing to log!", __PRETTY_FUNCTION__);
		return;
	}

	sqlite3* db;
	db = DBConnect();

	if (db == NULL) {
		Logger::error("%s Database is NULL!", __PRETTY_FUNCTION__);
		return;
	}

	Logger::info("Writing in the database last value: %s size logs %d",
	             logs[0].m_timestamp_str.c_str(), logs.size());

	// clang-format off
    actuatorFeedbackId  = getTableId("dataLogs_actuator_feedback");
    compassModelId      = getTableId("dataLogs_compass");
    courseCalculationId = getTableId("dataLogs_course_calculation");
    currentSensorsId    = getTableId("dataLogs_current_sensors");
    gpsId               = getTableId("dataLogs_gps");
    marineSensorsId     = getTableId("dataLogs_marine_sensors");
    vesselStateId       = getTableId("dataLogs_vessel_state");
    windStateId         = getTableId("dataLogs_wind_state");
    windsensorId        = getTableId("dataLogs_windsensor");
    // NOTE : Marc : To update the id of current_Mission in the DB
    currentMissionId    = getTableId("current_Mission");
    // clang-format on

	for (auto log : logs) {
		logNumber++;

		std::string sql;
		sqlite3_stmt* stmt = NULL;
		int retCode = 0;

		// sqlite3_exec(db, "BEGIN TRANSACTION;", NULL, NULL, &m_error);

		int _actuatorFeedbackId = 0;
		if (actuatorFeedbackId >= 0) {  // clang-format off
            if (!prepareStmtError(&stmt, "INSERT INTO "
                "dataLogs_actuator_feedback(rudder_position, wingsail_position, rc_on, wind_vane_angle, t_timestamp) "
                "VALUES(:rudder_position, :wingsail_position, :rc_on, :wind_vane_angle, :t_timestamp);"
            )) {  // clang-format on
	            bindParam(stmt, ":rudder_position", log.m_rudderPosition);
	            bindParam(stmt, ":wingsail_position", log.m_wingsailPosition);
	            bindParam(stmt, ":rc_on", log.m_radioControllerOn);
	            bindParam(stmt, ":wind_vane_angle", log.m_windVaneAngle);
	            bindParam(stmt, ":t_timestamp", log.m_timestamp_str);

	            if (stepAndFinalizeStmt(stmt) == SQLITE_DONE) {
		            _actuatorFeedbackId = actuatorFeedbackId + logNumber;
	            }
            }
		}

		int _compassModelId = 0;
		if (compassModelId >= 0) {  // clang-format off
            if (!prepareStmtError(&stmt, "INSERT INTO "
                "dataLogs_compass(heading, pitch, roll, t_timestamp) "
                "VALUES(:heading, :pitch, :roll, :t_timestamp);"
            )) {  // clang-format on
	            bindParam(stmt, ":heading", log.m_compassHeading);
	            bindParam(stmt, ":pitch", log.m_compassPitch);
	            bindParam(stmt, ":roll", log.m_compassRoll);
	            bindParam(stmt, ":t_timestamp", log.m_timestamp_str);

	            if (stepAndFinalizeStmt(stmt) == SQLITE_DONE) {
		            _compassModelId = compassModelId + logNumber;
	            }
            }
		}

		int _courseCalculationId = 0;
		if (courseCalculationId >= 0) {  // clang-format off
	        if (!prepareStmtError(&stmt, "INSERT INTO "
	            "dataLogs_course_calculation(distance_to_waypoint, bearing_to_waypoint, course_to_steer, tack, going_starboard, t_timestamp) "
	            "VALUES(:distance_to_waypoint, :bearing_to_waypoint, :course_to_steer, :tack, :going_starboard, :t_timestamp);"
	        )) {  // clang-format on
		        bindParam(stmt, ":distance_to_waypoint", log.m_distanceToWaypoint);
		        bindParam(stmt, ":bearing_to_waypoint", log.m_bearingToWaypoint);
		        bindParam(stmt, ":course_to_steer", log.m_courseToSteer);
		        bindParam(stmt, ":tack", log.m_tack);
		        bindParam(stmt, ":going_starboard", log.m_goingStarboard);
		        bindParam(stmt, ":t_timestamp", log.m_timestamp_str);
	        }
			if (stepAndFinalizeStmt(stmt) == SQLITE_DONE) {
				_courseCalculationId = courseCalculationId + logNumber;
			}
		}

		int _marineSensorsId = 0;
		if (marineSensorsId >= 0) {  // clang-format off
	    	if (!prepareStmtError(&stmt, "INSERT INTO "
				"dataLogs_marine_sensors(temperature, conductivity, ph, salinity, t_timestamp) "
	            "VALUES(:temperature, :conductivity, :ph, :salinity, :t_timestamp);"
		    )) {  // clang-format on
			    bindParam(stmt, ":temperature", log.m_temperature);
			    bindParam(stmt, ":conductivity", log.m_conductivity);
			    bindParam(stmt, ":ph", log.m_ph);
			    bindParam(stmt, ":salinity", log.m_salinity);
			    bindParam(stmt, ":t_timestamp", log.m_timestamp_str);
		    }
			if (stepAndFinalizeStmt(stmt) == SQLITE_DONE) {
				_marineSensorsId = marineSensorsId + logNumber;
			}
		}

		int _vesselStateId = 0;
		if (vesselStateId >= 0) {  // clang-format off
	    	if (!prepareStmtError(&stmt, "INSERT INTO "
	            "dataLogs_vessel_state(heading, latitude, longitude, speed, course, t_timestamp) "
			    "VALUES(:heading, :latitude, :longitude, :speed, :course, :t_timestamp);"
		    )) {  // clang-format on
			    bindParam(stmt, ":heading", log.m_vesselHeading);
			    bindParam(stmt, ":latitude", log.m_vesselLat);
			    bindParam(stmt, ":longitude", log.m_vesselLon);
			    bindParam(stmt, ":speed", log.m_vesselSpeed);
			    bindParam(stmt, ":course", log.m_vesselCourse);
			    bindParam(stmt, ":t_timestamp", log.m_timestamp_str);
		    }
			if (stepAndFinalizeStmt(stmt) == SQLITE_DONE) {
				_vesselStateId = vesselStateId + logNumber;
			}
		}

		int _windStateId = 0;
		if (windStateId >= 0) {  // clang-format off
	    	if (!prepareStmtError(&stmt, "INSERT INTO "
	            "dataLogs_wind_state(true_wind_speed, true_wind_direction, apparent_wind_speed, apparent_wind_direction, t_timestamp) "
			    "VALUES(:true_wind_speed, :true_wind_direction, :apparent_wind_speed, :apparent_wind_direction, :t_timestamp);"
		    )) {  // clang-format on
			    bindParam(stmt, ":true_wind_speed", log.m_trueWindSpeed);
			    bindParam(stmt, ":true_wind_direction", log.m_trueWindDir);
			    bindParam(stmt, ":apparent_wind_speed", log.m_apparentWindSpeed);
			    bindParam(stmt, ":apparent_wind_direction", log.m_apparentWindDir);
			    bindParam(stmt, ":t_timestamp", log.m_timestamp_str);
		    }
			if (stepAndFinalizeStmt(stmt) == SQLITE_DONE) {
				_windStateId = windStateId + logNumber;
			}
		}

		int _windsensorId = 0;
		if (windsensorId >= 0) {  // clang-format off
	    	if (!prepareStmtError(&stmt, "INSERT INTO "
	            "dataLogs_windsensor(direction, speed, temperature, t_timestamp) "
			    "VALUES(:direction, :speed, :temperature, :t_timestamp);"
		    )) {  // clang-format on
			    bindParam(stmt, ":direction", log.m_windDir);
			    bindParam(stmt, ":speed", log.m_windSpeed);
			    bindParam(stmt, ":temperature", log.m_windTemp);
			    bindParam(stmt, ":t_timestamp", log.m_timestamp_str);
		    }
			if (stepAndFinalizeStmt(stmt) == SQLITE_DONE) {
				_windsensorId = windsensorId + logNumber;
			}
		}

		int _gpsId = 0;
		if (gpsId >= 0) {  // clang-format off
	    	if (!prepareStmtError(&stmt, "INSERT INTO "
	            "dataLogs_gps (has_fix, online, time, latitude, longitude, speed, course, satellites_used, route_started, t_timestamp) "
			    "VALUES(:has_fix, :online, :time, :latitude, :longitude, :speed, :course, :satellites_used, :route_started, :t_timestamp);"
		    )) {  // clang-format on
			    bindParam(stmt, ":has_fix", log.m_gpsHasFix);
			    bindParam(stmt, ":online", log.m_gpsOnline);
			    bindParam(stmt, ":time",
			              log.m_timestamp_str);  // TODO: This could be the GPS time instead
			    bindParam(stmt, ":latitude", log.m_gpsLat);
			    bindParam(stmt, ":longitude", log.m_gpsLon);
			    bindParam(stmt, ":speed", log.m_gpsSpeed);
			    bindParam(stmt, ":course", log.m_gpsCourse);
			    bindParam(stmt, ":satellites_used", log.m_gpsSatellite);
			    bindParam(stmt, ":route_started", log.m_routeStarted);
			    bindParam(stmt, ":t_timestamp", log.m_timestamp_str);
		    }
			if (stepAndFinalizeStmt(stmt) == SQLITE_DONE) {
				_gpsId = gpsId + logNumber;
			}
		}

		int _currentSensorsId = 0;
		if (currentSensorsId >= 0) {  // clang-format off
	    	if (!prepareStmtError(&stmt, "INSERT INTO "
	            "dataLogs_current_sensors(current, voltage, element, element_str, t_timestamp) "
			    "VALUES(:current, :voltage, :element, :element_str, :t_timestamp);"
		    )) {  // clang-format on
			    bindParam(stmt, ":current", log.m_current);
			    bindParam(stmt, ":voltage", log.m_voltage);
			    bindParam(stmt, ":element", log.m_element);
			    bindParam(stmt, ":element_str", log.m_element_str);
			    bindParam(stmt, ":t_timestamp", log.m_timestamp_str);
		    }
			if (stepAndFinalizeStmt(stmt) == SQLITE_DONE) {
				_currentSensorsId = currentSensorsId + logNumber;
			}
		}

		// clang-format off
	    retCode = prepareStmtError(&stmt, "INSERT INTO "
			"dataLogs_system(actuator_feedback_id, compass_id, course_calculation_id, current_sensors_id, gps_id, marine_sensors_id, vessel_state_id, wind_state_id, windsensor_id, current_mission_id) "
            "VALUES(:actuator_feedback_id, :compass_id, :course_calculation_id, :current_sensors_id, :gps_id, :marine_sensors_id, :vessel_state_id, :wind_state_id, :windsensor_id, :current_mission_id);"
	    );
        if (!retCode) {  // clang-format on
	        bindParam(stmt, ":actuator_feedback_id", _actuatorFeedbackId);
	        bindParam(stmt, ":compass_id", _compassModelId);
	        bindParam(stmt, ":course_calculation_id", _courseCalculationId);
	        bindParam(stmt, ":current_sensors_id", _currentSensorsId);
	        bindParam(stmt, ":gps_id", _gpsId);
	        bindParam(stmt, ":marine_sensors_id", _marineSensorsId);
	        bindParam(stmt, ":vessel_state_id", _vesselStateId);
	        bindParam(stmt, ":wind_state_id", _windStateId);
	        bindParam(stmt, ":windsensor_id", _windsensorId);
	        bindParam(stmt, ":current_mission_id", currentMissionId);

	        if (stepAndFinalizeStmt(stmt) == SQLITE_DONE) {
		        m_latestDataLogId = getTableId("dataLogs_system");
	        } else {
		        m_latestDataLogId = 0;
		        Logger::error("%s Error, failed to create system log index!", __PRETTY_FUNCTION__);
	        }
        }
		// do {
		//    retCode = sqlite3_exec(db, "END TRANSACTION;", NULL, NULL, &m_error);
		//} while (retCode == SQLITE_BUSY);

		if (retCode != SQLITE_OK) {
			Logger::error("%s SQLite COMMIT error: %s (%d)", __PRETTY_FUNCTION__,
			              sqlite3_errstr(retCode), retCode);
			if (m_error != NULL) {
				Logger::error("%s SQLite DB error: %s", __PRETTY_FUNCTION__, sqlite3_errmsg(db));

				sqlite3_free(m_error);
				m_error = NULL;
			}
			//            retCode = sqlite3_exec(db, "ROLLBACK TRANSACTION;", NULL, NULL,
			//            &m_error); Logger::info("%s SQLite ROLLBACK returned %s (%d)",
			//            __PRETTY_FUNCTION__,
			//                         sqlite3_errstr(retCode), retCode);
		}
	}
	DBDisconnect();
}



/*int DBHandler::insertLog(std::string table, std::string values) {
    std::stringstream ss;
    ss << "INSERT INTO " << table << " VALUES(NULL, " << values << ");";

    if (DBTransaction(ss.str())) {
        Timer time_;
        time_.start();
        int tableId = getTableId(table);
        time_.stop();
        Logger::info("Time passed writing %.5f", time_.timePassed());
        if (tableId > 0) {
            return tableId;
        }
    }
    Logger::error("%s Error, failed to insert log Request: %s", __PRETTY_FUNCTION__,
                  ss.str().c_str());
    return 0;
}*/
























/******************************************************************************
 * DBTransaction() - Atomic transaction, all or nothing gets done
 * @param SQLQuery
 * @return bool
 */
bool DBHandler::DBTransaction(std::string SQLQuery) {
    sqlite3* db = DBConnect();
    m_error = NULL;

    if (db != NULL) {
        int resultcode = 0;

        do {
            if (m_error != NULL) {
                sqlite3_free(m_error);
                m_error = NULL;
            }
            sqlite3_exec(db, "BEGIN TRANSACTION", NULL, NULL, &m_error);
            resultcode = sqlite3_exec(db, SQLQuery.c_str(), NULL, NULL, &m_error);
            sqlite3_exec(db, "END TRANSACTION", NULL, NULL, &m_error);
        } while (resultcode == SQLITE_BUSY);

        if (m_error != NULL) {
            Logger::error("%s Error: %s", __PRETTY_FUNCTION__, sqlite3_errmsg(db));

            sqlite3_free(m_error);
            return false;
        }
    } else {
        Logger::error("%s Error: no database found", __PRETTY_FUNCTION__);
        return false;
    }
    return true;
}

/*std::vector<std::string> DBHandler::getTableIds(std::string table) {
    int rows, columns;
    std::vector<std::string> results;
    try {
        results = retrieveFromTable("SELECT id FROM " + table + ";", rows, columns);
    } catch (const char* error) {
    }

    std::vector<std::string> ids;
    for (int i = 1; i <= rows; i++) {
        ids.push_back(results[i]);
    }

    return ids;
}*/

// TODO: Rewrite old
std::vector<std::string> DBHandler::getTableNames(std::string like) {
    int rows, columns;
    std::vector<std::string> results;
    try {
        results = retrieveFromTable(
            "SELECT name FROM sqlite_master WHERE type='table' AND name LIKE '" + like + "';", rows,
            columns);
    } catch (const char* error) {
    }

    std::vector<std::string> tableNames;
    for (unsigned int i = 1; i < results.size(); i++) {
        tableNames.push_back(results[i]);
    }
    return tableNames;
}

// TODO: Rewrite old
std::vector<std::string> DBHandler::getColumnInfo(std::string info, std::string table) {
    int rows = 0, columns = 0;
    std::vector<std::string> results;

    std::string pragmaQuery = "PRAGMA table_info(" + table + ");";

    try {
        results = retrieveFromTable(pragmaQuery, rows, columns);
    } catch (const char* error) {
    }

    std::vector<std::string> types;
    int infoIndex = 0;
    for (int i = 0; i < columns; i++) {
        if (std::string(info).compare(results[i]) == 0) {
            infoIndex = i;
        }
    }

    for (int i = 1; i < rows + 1; i++) {
        types.push_back(results[i * columns + infoIndex]);
    }
    return types;
}

// TODO: Rewrite old
bool DBHandler::getWaypointValues(int& nextId,
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
                                  bool& foundPrev) {
    int rows, columns, rows2, columns2;
    std::vector<std::string> results;
    std::vector<std::string> results2;
    try {
        results = retrieveFromTable("SELECT MIN(id) FROM current_Mission WHERE harvested = 0;",
                                    rows, columns);
        results2 = retrieveFromTable("SELECT MAX(id) FROM current_Mission WHERE harvested = 1;",
                                     rows2, columns2);
    } catch (const char* error) {
        Logger::error("%s Error: %s", __PRETTY_FUNCTION__, error);
        return false;
    }

    if (rows * columns < 1 || results[1] == "\0") {
        return false;
    }
    // Do not give values to previous waypoint if no value found in database
    foundPrev = true;
    if (rows2 * columns2 < 1 || results2[1] == "\0") {
        Logger::info("No previously harvested waypoint found, values set as 0");
        foundPrev = false;
    }

    // Set values to next waypoint
    nextId = safe_stoi(results[1]);
    selectFromId(nextLongitude, "longitude", "current_Mission", safe_stoi(results[1]));
    selectFromId(nextLatitude, "latitude", "current_Mission", safe_stoi(results[1]));
    selectFromId(nextDeclination, "declination", "current_Mission", safe_stoi(results[1]));
    selectFromId(nextRadius, "radius", "current_Mission", safe_stoi(results[1]));
    selectFromId(nextStayTime, "stay_time", "current_Mission", safe_stoi(results[1]));
    selectFromId(isCheckpoint, "is_checkpoint", "current_Mission", safe_stoi(results[1]));

    if (foundPrev) {  // Set values to next waypoint if harvested waypoint found
        prevId = safe_stoi(results2[1]);
        selectFromId(prevLongitude, "longitude", "current_Mission", safe_stoi(results2[1]));
        selectFromId(prevLatitude, "latitude", "current_Mission", safe_stoi(results2[1]));
        selectFromId(prevDeclination, "declination", "current_Mission", safe_stoi(results2[1]));
        selectFromId(prevRadius, "radius", "current_Mission", safe_stoi(results2[1]));
    }

    return true;
}



/*
int DBHandler::stmtSelectFrom(sqlite3_stmt** stmt,
                              const std::string& selector,
                              const std::string& from,
                              const int id) {
    sqlite3* db = DBConnect();
    int retCode = -1;
    std::string sql = "SELECT " + selector + " FROM " + from;
    if (id) {
        sql += " WHERE id = :id";
    }

    if (!prepareStmtError(db, sql, stmt)) {
        if (id) {
            bindParam(*stmt, ":id", id);
        }

        retCode = sqlite3_step(*stmt);
        if (retCode != SQLITE_ROW) {
            Logger::error("%s SQLite error: %s (%d) on \"%s\"", __PRETTY_FUNCTION__,
                          sqlite3_errstr(retCode), retCode, sql.c_str());
        }
    }
    return retCode;
}
*/


/*std::string DBHandler::selectFromAsText(std::string table, std::string column, std::string id) {
    std::string query = "SELECT " + column + " FROM " + table + " WHERE id=" + id + ";";

    int rows, columns;
    std::vector<std::string> results;
    try {
        results = retrieveFromTable(query, rows, columns);
    } catch (const char* error) {
        rows = 0;
        columns = 0;
    }

    if (columns < 1) {
        Logger::error("%s No columns from Query: %s", __PRETTY_FUNCTION__, query.c_str());
        return "";
    }

    if (rows < 1) {
        Logger::error("%s No rows from Query: %s", __PRETTY_FUNCTION__, query.c_str());
        return "";
    }

    return results[1];
}*/

/*
int DBHandler::stmtSelectFrom(std::string table, std::string column, int id) {
    std::string data = selectFromAsText(table, id, column);
    if (data.size() > 0) {
        return strtol(data.c_str(), NULL, 10);
    } else {
        Logger::error("%s, Error: No data in cell ", __PRETTY_FUNCTION__);
        return 0;
    }
}
*/

/*
double DBHandler::getConfigsFrom(std::string table, std::string column, std::string id) {
    std::string data = selectFromAsText(table, id, column);
    if (data.size() > 0) {
        return strtod(data.c_str(), NULL);
    } else {
        Logger::error("%s, Error: No data in cell ", __PRETTY_FUNCTION__);
        return 0;
    }
}*/

/*std::string DBHandler::getLogs(bool onlyLatest) {
    JSON js;

    // fetch all datatables starting with "dataLogs_"
    std::vector<std::string> datalogTables = getTableNames("dataLogs_%");
    std::vector<std::vector<std::string>> rows = rowsAsText()
    try {
        // insert all data in these tables as json array

        for (auto table : datalogTables) {
            if (onlyLatest) {
                // Gets the log entry with the highest id
                getDataAsJson("*", table + " ORDER BY id DESC LIMIT 1", table, "", js, true);
            } else {
                getDataAsJson("*", table, table, "", js, true);
            }

        }

    } catch (const char* error) {
        Logger::error("%s, Error: %s", __PRETTY_FUNCTION__, error);
    }
    return js.dump();
}*/

std::string DBHandler::getLogs(bool onlyLatest) {
	sqlite3_stmt** stmt = NULL;
    std::string result;
    JSON js;

    // fetch all datatables starting with "dataLogs_"
    std::vector<std::string> datalogTables = getTableNames("dataLogs_%");
    std::vector<std::tuple<std::string, std::vector<std::vector<std::string>>>> logs;

    try {
        // insert all data in these tables as json array

        for (auto table : datalogTables) {
	        // Gets the log entry with the highest id if flag is set
            prepareStmtSelectFromStatements(stmt, "*", table, (onlyLatest ? "ORDER BY id DESC LIMIT 1" : NULL));
            std::vector<std::vector<std::string>> rows = rowsAsText(stmt);

			prepareStmtSelectFromStatements(stmt, "name", "sqlite_master", "WHERE type='table' AND name LIKE " + table);
            std::vector<std::string> names = rowsAsText(stmt)[0];

            // "dataLogs_" is 9 chars, next is at index 9 (starting from 0)
            logs.push_back(std::make_tuple(table.substr(9, std::string::npos), rows));
            // getDataAsJson("*", table, table, "", js, true);
        }

    } catch (const char* error) {
        Logger::error("%s, Error gathering log data %s", __PRETTY_FUNCTION__, error);
    }

    try {
        int rowCnt = 0;
        for (auto tup : logs) {
            std::string table = std::get<0>(tup);
            std::vector<std::vector<std::string>> rows = std::get<1>(tup);

            for (auto row : rows) {
                js[table.c_str()].push_back(row);
                rowCnt++;
            }
        }
        // No rows will return an empty string
        if (rowCnt)
            result = js.dump();
    } catch (const char* error) {
        Logger::error("%s, Error JSON-encoding data %s", __PRETTY_FUNCTION__, error);
    }
    datalogTables.clear();
    logs.clear();
    return result;
}

// Kill
void DBHandler::getDataAsJson(std::string select,
                              std::string table,
                              std::string key,
                              std::string id,
                              JSON& js,
                              bool useArray) {
    int rows = 0, columns = 0;
    std::vector<std::string> values;
    std::vector<std::string> columnNames;
    std::vector<std::string> results;

    try {
        if (id == "") {
            results = retrieveFromTable("SELECT " + select + " FROM " + table + ";", rows, columns);
        } else {
            results = retrieveFromTable(
                "SELECT " + select + " FROM " + table + " WHERE ID = " + id + ";", rows, columns);
        }
    } catch (const char* error) {
        Logger::error("%s, %s table: %s Error: %s", __PRETTY_FUNCTION__, select.c_str(),
                      table.c_str(), error);
    }

    for (int i = 0; i < columns * (rows + 1); ++i) {
        if (i < columns) {
            columnNames.push_back(results[i]);
        } else {
            values.push_back(results[i]);
        }
    }

    JSON jsonEntry;
    for (int i = 0; i < rows; i++) {
        for (unsigned int j = 0; j < columnNames.size(); j++) {
            int index = j + (columns * i);
            jsonEntry[columnNames.at(j)] = values.at(index);
        }

        if (useArray) {
            if (!js[key].is_array()) {
                js[key] = JSON::array({});
            }
            js[key].push_back(jsonEntry);
        } else {
            js[key] = jsonEntry;
        }
    }

    values.clear();
    columnNames.clear();
}

// TODO: Rewrite old
std::vector<std::string> DBHandler::retrieveFromTable(std::string SQLSelect,
                                                      int& rows,
                                                      int& columns) {
    std::vector<std::string> results;
    int resultCode = 0;

    do {
        resultCode = getTable(SQLSelect, results, rows, columns);
    } while (resultCode == SQLITE_BUSY);

    if (!((resultCode == SQLITE_OK) || resultCode == SQLITE_EMPTY)) {
        Logger::error("%s SQL statement: %s Error: %s", __PRETTY_FUNCTION__, SQLSelect.c_str(),
                      sqlite3_errstr(resultCode));
        throw "DBHandler::retrieveFromTable() SQL error";
    }
    return results;
}

// TODO: Rewrite old
int DBHandler::getTable(const std::string& sql,
                        std::vector<std::string>& results,
                        int& rows,
                        int& columns) {
    int resultCode = -1;
    sqlite3_stmt* statement;
    sqlite3* db = DBConnect();

    // prepare the statement sql code in byte form for query request
    if ((resultCode = sqlite3_prepare_v2(db, sql.c_str(), sql.size(), &statement, NULL)) !=
        SQLITE_OK) {  // if not OK, return error
        sqlite3_finalize(statement);
        return resultCode;
    }
    // TODO: No step?!!!?!???!??

    // get the number of columns int the table called in the statement
    columns = sqlite3_column_count(statement);
    rows = 0;

    // read column names
    for (int i = 0; i < columns; i++) {
        // if column name is NULL, return error
        if (!sqlite3_column_name(statement, i)) {
            sqlite3_finalize(statement);
            return SQLITE_EMPTY;
        }

        // add to the result
        results.emplace_back(const_cast<char*>(sqlite3_column_name(statement, i)));
    }

    // read the rest of the table
    while ((resultCode = sqlite3_step(statement)) == SQLITE_ROW) {
        for (int i = 0; i < columns; i++) {
            if (results[i] != "dflt_value")  //[es] Not a perfect solution. Needed for pragma sql
                                             // statements as it is always null
            {
                // Get the value in the column
                if (!sqlite3_column_text(statement, i)) {
                    sqlite3_finalize(statement);
                    rows = 0;
                    columns = 0;
                    return SQLITE_EMPTY;
                }

                results.emplace_back(reinterpret_cast<char*>(
                    const_cast<unsigned char*>(sqlite3_column_text(statement, i))));
            } else {
                results.emplace_back("NULL");
            }
        }
        rows++;
    }

    sqlite3_finalize(statement);  // destruct the statement

    if (resultCode != SQLITE_DONE) {
        return resultCode;
    }
    return SQLITE_OK;
}
