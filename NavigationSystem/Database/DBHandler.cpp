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
    return (connection != nullptr);  // NULL would mean we got no connection
}

/******************************************************************************
 * private helpers
 *****************************************************************************/

/**
 * Checks and logs possible SQLite error codes
 * @param SQLite error code
 * @return SQLite error code
 */
int DBHandler::checkRetCode(const int retCode) const {
    if (not((retCode == SQLITE_OK) || (retCode == SQLITE_DONE) || (retCode == SQLITE_ROW))) {
        Logger::error("SQLite result code: %s (%d)", sqlite3_errstr(retCode), retCode);
    }
    return retCode;
}

/**
 * sqlite3_step() and sqlite3_finalize() helper function
 * @param stmt
 * @return SQLite error code
 */
int DBHandler::stepAndFinalizeStmt(sqlite3_stmt*& stmt) const {
    int retCode = checkRetCode(sqlite3_step(stmt));
    checkRetCode(sqlite3_finalize(stmt));  // Only for logging possible error
    return retCode;                        // Note that this is the retCode from sqlite3_step
}

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
        if (db_file == nullptr) {
            Logger::error("%s %s not found", __PRETTY_FUNCTION__, m_filePath.c_str());
            m_databaseLock.unlock();
            return nullptr;
        }
        fclose(db_file);

        do {
            resultcode = sqlite3_open(m_filePath.c_str(), &m_DBHandle);
        } while (resultcode == SQLITE_BUSY);

        if (resultcode) {
            Logger::error("%s Failed to open the database Error %s", __PRETTY_FUNCTION__,
                          sqlite3_errmsg(m_DBHandle));
            m_databaseLock.unlock();
            return nullptr;
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
    if (m_DBHandle != nullptr) {
        Logger::info("%s closing Database", __PRETTY_FUNCTION__);
        sqlite3_close(m_DBHandle);
        m_DBHandle = nullptr;
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

/**
 *
 * @param db
 * @param sql
 * @param stmt
 * @return
 */
int DBHandler::prepareStmtError(sqlite3_stmt*& stmt, std::string sql) {
    sqlite3* db = DBConnect();
    int resultCode =
        checkRetCode(sqlite3_prepare_v2(db, sql.c_str(), (int)sql.size(), &stmt, nullptr));
    if (resultCode != SQLITE_OK) {
        Logger::error("%s: %s (%d) on \"%s\"", __PRETTY_FUNCTION__, sqlite3_errstr(resultCode),
                      resultCode, sql.c_str());
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
int DBHandler::prepareStmtSelectFromStatements(sqlite3_stmt*& stmt,
                                               const std::string& expressions,
                                               const std::string& tables,
                                               const std::string& statements) {
    std::string sql = "SELECT " + expressions + " FROM " + tables;
    if (statements.length()) {
        sql += " " + statements;
    }
    // sql += ";";  // Not really needed but neat
    return prepareStmtError(stmt, sql);
}

/*int DBHandler::prepareStmtUpdateError(sqlite3_stmt *&stmt, const std::string& tables, std::string&
expressions, std::string& statements) { std::string sql = "UPDATE " + tables + " SET " +
expressions; if (statements.length()) { sql += " " + statements;
    }
    return prepareStmtError(stmt, sql);
}*/

int DBHandler::prepareStmtInsertError(sqlite3_stmt*& stmt,
                                      const std::string& table,
                                      std::string& columns,
                                      std::string& values) {
    std::string sql = "INSERT INTO " + table + "(" + columns + ") VALUES(" + values + ")";
    // sql += ";";  // Not really needed but neat
    return prepareStmtError(stmt, sql);
}

int DBHandler::prepareStmtInsertError(sqlite3_stmt*& stmt,
                                      const std::string& table,
                                      std::vector<std::string>& columns) {
    std::string columnString = joinStrings(columns, ",");
    std::string valueString = joinStrings(prependStrings(columns, ":"), ",");
    return prepareStmtInsertError(stmt, table, columnString, valueString);
}

int DBHandler::prepareStmtInsertError(sqlite3_stmt*& stmt,
                                      const std::string& table,
                                      bindValues& values) {
    std::vector<std::string> names = valueNames(values);
    return prepareStmtInsertError(stmt, table, names);
}


int DBHandler::prepareStmtUpdateError(sqlite3_stmt *&stmt, const std::string &table, int id, std::string &columns,
                                      std::string &values) {
    std::string sql = "UPDATE " + table + "(" + columns + ") VALUES(" + values + ") WHERE id = :id";
    // sql += ";";  // Not really needed but neat
    int retCode = prepareStmtError(stmt, sql);
    if (retCode == SQLITE_OK) {
    	retCode = bindParam(stmt, ":id", id);
    }
    return retCode;
}

int DBHandler::prepareStmtUpdateError(sqlite3_stmt *&stmt, const std::string &table, int id,
                                      std::vector<std::string> &columns) {
    std::string columnString = joinStrings(columns, ",");
    std::string valueString = joinStrings(prependStrings(columns, ":"), ",");
    return prepareStmtUpdateError(stmt, table, id, columnString, valueString);
}

int DBHandler::prepareStmtUpdateError(sqlite3_stmt *&stmt, const std::string &table, int id, bindValues &values) {
    std::vector<std::string> names = valueNames(values);
    return prepareStmtUpdateError(stmt, table, id, names);
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
int DBHandler::bindParam(sqlite3_stmt*& stmt, const char* name, const int& value) {
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
int DBHandler::bindParam(sqlite3_stmt*& stmt, const char* name, const double& value) {
    int paramIndex = paramNameIndex(stmt, name);
    if (!paramIndex) {
        return SQLITE_MISUSE;
    }
    return checkRetCode(sqlite3_bind_double(stmt, paramIndex, value));
}

/**
 * Binds a string to a statement
 * @param stmt		SQLite statement
 * @param name		Parameter name
 * @param value		Value
 * @return
 */
int DBHandler::bindParam(sqlite3_stmt*& stmt, const char* name, const std::string& text) {
    int paramIndex = paramNameIndex(stmt, name);
    if (!paramIndex) {
        return SQLITE_MISUSE;
    }
    return checkRetCode(sqlite3_bind_text(stmt, paramIndex, text.c_str(),
                                          static_cast<int>(text.size()), SQLITE_TRANSIENT));
}

/**
 * Returns the numerical index of a named parameter
 * @param stmt
 * @param name
 * @return
 */
int DBHandler::paramNameIndex(sqlite3_stmt*& stmt, const char* name) {
    int paramIndex = sqlite3_bind_parameter_index(stmt, name);
    if (!paramIndex) {
        paramIndex = sqlite3_bind_parameter_index(
            stmt, prependString(name, ":").c_str());  // retry it with an added :
        if (paramIndex) {
            Logger::warning("Added ':' to %s to make it work", name);
        }
    }
    if (!paramIndex) {
        Logger::error("SQLite null parameter index on \"%s\"!", name);
    }
    return paramIndex;
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
    sqlite3_stmt*& stmt,
    const std::vector<std::pair<const char*, int>>& ints,
    const std::vector<std::pair<const char*, double>>& doubles,
    const std::vector<std::pair<const char*, std::string>>& strings) {
    int firstErrorCode = SQLITE_OK;
    int retCode;  // Uggly but works

    for (auto i : ints) {
        firstErrorCode = bindParam(stmt, i.first, i.second);
    }
    for (auto d : doubles) {
        retCode = bindParam(stmt, d.first, d.second);
        if (firstErrorCode == SQLITE_OK)
            firstErrorCode = retCode;
    }
    for (auto s : strings) {
        retCode = bindParam(stmt, s.first, s.second);
        if (firstErrorCode == SQLITE_OK)
            firstErrorCode = retCode;
    }
    return firstErrorCode;
}

int DBHandler::bindValuesToStmt(const bindValues& values, sqlite3_stmt*& stmt) {
    int firstErrorCode = SQLITE_OK;
    int retCode;  // Uggly but works

    // This could probably be optimized
    for (auto pair : values.ints) {
        retCode = bindParam(stmt, prependString(pair.first, ":").c_str(), pair.second);
        if ((firstErrorCode == SQLITE_OK) && (retCode != SQLITE_OK)) {
            firstErrorCode = retCode;
        }
    }
    for (auto pair : values.doubles) {
        retCode = bindParam(stmt, prependString(pair.first, ":").c_str(), pair.second);
        if ((firstErrorCode == SQLITE_OK) && (retCode != SQLITE_OK)) {
            firstErrorCode = retCode;
        }
    }
    for (auto pair : values.strings) {
        retCode = bindParam(stmt, prependString(pair.first, ":").c_str(), pair.second);
        if ((firstErrorCode == SQLITE_OK) && (retCode != SQLITE_OK)) {
            firstErrorCode = retCode;
        }
    }
    return firstErrorCode;
}

void DBHandler::addValue(bindValues& values, const char* name, int value) {
    std::pair<const char*, int> pair = std::make_pair(name, value);
    values.ints.emplace_back(pair);
}
void DBHandler::addValue(bindValues& values, const char* name, double value) {
    std::pair<const char*, double> pair = std::make_pair(name, value);
    values.doubles.emplace_back(pair);
}
void DBHandler::addValue(bindValues& values, const char* name, std::string& string) {
    std::pair<const char*, std::string> pair = std::make_pair(name, string);
    values.strings.emplace_back(pair);
}

std::vector<std::string> DBHandler::valueNames(const bindValues& values) {
    std::vector<std::string> names;

    // This could probably be optimized
    for (auto pair : values.ints)
        names.emplace_back(pair.first);
    for (auto pair : values.doubles)
        names.emplace_back(pair.first);
    for (auto pair : values.strings)
        names.emplace_back(pair.first);
    return std::move(names);
}

/*******************************************************************************
 * Functions for retreiving data from SQLite DB
 ******************************************************************************/

// TODO: BUSY loop
/**
 * SELECT vector of rows as vectors of columns as strings
 * @param stmt
 * @param rowHeader	Makes row 0 of all tables contain the column names
 * @return
 */
std::vector<std::vector<std::string>> DBHandler::getRowsAsText(sqlite3_stmt*& stmt,
                                                               bool rowHeader) {
    std::vector<std::vector<std::string>> rows;
    int retCode = sqlite3_step(stmt);
    if (not((retCode == SQLITE_ROW) || (retCode == SQLITE_DONE))) {
        Logger::error("%s SQLite error: %s (%d)", __PRETTY_FUNCTION__, sqlite3_errstr(retCode));
    } else {
        int columns = sqlite3_column_count(stmt);
        std::vector<std::string> row;
        if (rowHeader) {
            for (int i = 0; i < columns; i++) {
                row.emplace_back(sqlite3_column_name(stmt, i));
            }
            rows.push_back(row);
            row.clear();
        }
        while (retCode == SQLITE_ROW) {
            for (int i = 0; i < columns; i++) {
                std::string retStr;
                sqlite3_column_value(stmt, i, retStr);
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
            row.clear();
            retCode = sqlite3_step(stmt);
            checkRetCode(retCode);  // TODO inline above
        }
    }
    // checkRetCode(sqlite3_finalize(stmt));
    return std::move(rows);
}

/**
 * Matches DB table names
 * @param like
 * @return a vector of strings
 */
std::vector<std::string> DBHandler::getTableNames(std::string like) {
    sqlite3_stmt* stmt = nullptr;
    std::vector<std::vector<std::string>> results;
    std::vector<std::string> tableNames;

    prepareStmtSelectFromStatements(stmt, "name", "sqlite_master",
                                    "WHERE type='table' AND name LIKE :like");
    bindParam(stmt, ":like", like);

    results = getRowsAsText(stmt);
    for (auto result : results) {
        tableNames.push_back(result[0]);
    }
    return tableNames;
}

/**
 * get id from table returns either max or min id from table
 * @param table
 * @param minmax
 * @return
 */
int DBHandler::getTableId(const char* table, ID_MINMAX minmax) {
    int retCode;
    sqlite3_stmt* stmt = nullptr;
    int id = 0;
    std::string selector = (minmax ? "MAX(id)" : "MIN(id)");

    retCode = prepareStmtSelectFromStatements(stmt, selector, table, "");
    if (retCode == SQLITE_OK)
        retCode = sqlite3_step(stmt);
    if (retCode == SQLITE_ROW) {
        sqlite3_column_value(stmt, 0, id);
        retCode = SQLITE_OK;
    }
    if (retCode != SQLITE_OK) {
        Logger::error("%s Error determining %s from %s", __PRETTY_FUNCTION__, selector.c_str(),
                      table);
    }
    if (stmt != nullptr)
        sqlite3_finalize(stmt);
    return id;
}

// TODO: Rewrite old
// Not sure if this is even needed
std::vector<std::string> DBHandler::getColumnInfo(std::string info, std::string table) {
    int rows = 0, columns = 0;
    std::vector<std::string> results;
    std::string sql = "PRAGMA table_info(" + table + ")";

    sqlite3_stmt* stmt;
    if (!prepareStmtError(stmt, sql)) {
        std::vector<std::string> results2 = getRowsAsText(stmt)[0];
    }

    try {
        results = retrieveFromTable(sql, rows, columns);
    } catch (const char* error) {
    }

    std::vector<std::string> types;
    int infoIndex = 0;
    for (int i = 0; i < columns; i++) {
        if (std::string(info) == results[i]) {
            infoIndex = i;
        }
    }

    for (int i = 1; i < rows + 1; i++) {
        types.push_back(results[i * columns + infoIndex]);
    }
    return types;
}

// TODO: Rewrite old - this is used by WaypointMgrNode
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
	int nextWayPointId = -1;
	int prevWayPointId = -1;
	sqlite3_stmt *stmt;
	int retCode = prepareStmtSelectFromStatements(stmt, "MIN(id)", "current_Mission", "WHERE harvested = 0");
	if (retCode == SQLITE_OK) {
		retCode = sqlite3_step(stmt);
		if (retCode == SQLITE_ROW) {
			sqlite3_column_value(stmt, 0, nextWayPointId);
		}
		prevWayPointId = ( nextWayPointId > 0 ? nextWayPointId-1 : 0 );
	}

	if (retCode != SQLITE_OK) {
		Logger::error("%s Error: %s", __PRETTY_FUNCTION__, error);
		return false;
	}


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

/**
 * Retreives a table as a vector of tuples tablename - vector of rows (with 0 being a header)
 * converted to JSON
 * @param like
 * @param statement
 * @return
 */
std::string DBHandler::getTablesAsJSON(std::string like, std::string statement) {
    sqlite3_stmt* stmt = nullptr;
    std::string result;
    JSON js;

    std::vector<std::string> tableNames = getTableNames(like);
    std::vector<std::tuple<std::string, std::vector<std::vector<std::string>>>> tableContents;

    try {
        // insert all data in these tables as json array
        for (const auto& table : tableNames) {
            prepareStmtSelectFromStatements(stmt, "*", table, statement);
            std::vector<std::vector<std::string>> rows = getRowsAsText(stmt, true);
            sqlite3_finalize(stmt);

            if (rows.size() > 1) {  // we want actual data, not only the headers
                // "dataLogs_" is 9 chars, next is at index 9 (starting from 0). config_% Like minus
                // one will work check of like is a single or has % in it?
                int wildcards = 0;
                for (auto c : like) {
                    if (c == '%')
                        wildcards++;
                }
                std::string tableTitle =
                    (wildcards ? table.substr(like.length() - wildcards, std::string::npos)
                               : table);
                tableContents.emplace_back(tableTitle, rows);
            } else {
                Logger::warning("%s, Table %s empty", __PRETTY_FUNCTION__, table.c_str());
            }
        }

    } catch (const char* error) {
        Logger::error("%s, Error gathering data from %s: %s", __PRETTY_FUNCTION__, like, error);
    }

    try {
        int rowCnt = 0;
        for (auto tup : tableContents) {
            std::string table = std::get<0>(tup);
            std::vector<std::vector<std::string>> rows = std::get<1>(tup);

            for (auto row : rows) {
                js[table.c_str()].push_back(row);
                rowCnt++;
            }
        }
        // No rows will return an empty string
        if (rowCnt) {
            result = js.dump();
        }
    } catch (const char* error) {
        Logger::error("%s, Error JSON-encoding data from %s: %s", __PRETTY_FUNCTION__, like, error);
    }
    tableNames.clear();
    tableContents.clear();
    return result;
}

/**
 * Gets all configs as JSON
 * @return
 */
std::string DBHandler::getConfigs() {
    return getTablesAsJSON("config_%", "WHERE id = 1");
}

/**
 * Gets all dataLogs as JSON
 * @param onlyLatest
 * @return
 */
std::string DBHandler::getLogsAsJSON(bool onlyLatest) {
    return getTablesAsJSON("dataLogs_%", (onlyLatest ? "ORDER BY id DESC LIMIT 1" : nullptr));
}

std::string DBHandler::getWayPointsAsJSON() {
    std::string result = getTablesAsJSON("current_Mission");
    if (result.empty()) {
        Logger::warning("No waypoints in database");
    }
    return result;
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
    int resultCode;
    sqlite3_stmt* statement;
    sqlite3* db = DBConnect();

    // prepare the statement sql code in byte form for query request
    if ((resultCode = sqlite3_prepare_v2(db, sql.c_str(), static_cast<int>(sql.size()), &statement,
                                         nullptr)) != SQLITE_OK) {  // if not OK, return error
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

/*******************************************************************************
 * INSERT and UPDATE ops
 ******************************************************************************/

/**
 * Inserts all current values in the DB. Uses overloaded bindParam instead of
 * generic bining so you do not have to worry about types
 * @param logs Object contaning data to be logged
 */
void DBHandler::insertDataLogs(std::vector<LogItem>& logs) {
    int actuatorFeedbackId;
    int compassModelId;
    int courseCalculationId;
    int currentMissionId;
    int currentSensorsId;
    int gpsId;
    int marineSensorsId;
    int vesselStateId;
    int windStateId;
    int windsensorId;

    int logNumber = 0;
    std::string tableId;

    if (logs.empty()) {
        Logger::error("%s Nothing to log!", __PRETTY_FUNCTION__);
        return;
    }

    sqlite3* db;
    db = DBConnect();

    if (db == nullptr) {
        Logger::error("%s Database is NULL!", __PRETTY_FUNCTION__);
        return;
    }

    Logger::info("Writing in the database last value: %s size logs %d",
                 logs[0].m_timestamp_str.c_str(), logs.size());

    // clang-format off
    actuatorFeedbackId  = getTableId("dataLogs_actuator_feedback"); // "SELECT name FROM sqlite_master WHERE type='table' AND name LIKE dataLogs_actuator_feedback"
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
        sqlite3_stmt* stmt = nullptr;
        // int retCode = 0;

        // sqlite3_exec(db, "BEGIN TRANSACTION;", NULL, NULL, &m_error);

        int _actuatorFeedbackId = 0;
        if (actuatorFeedbackId) {
            bindValues values;
            addValue(values, "rudder_position", log.m_rudderPosition);
            addValue(values, "wingsail_position", log.m_wingsailPosition);
            addValue(values, "rc_on", log.m_radioControllerOn);
            addValue(values, "wind_vane_angle", log.m_windVaneAngle);
            addValue(values, "t_timestamp", log.m_timestamp_str);
            if (!prepareStmtInsertError(stmt, "dataLogs_actuator_feedback", values)) {
                bindValuesToStmt(values, stmt);
                if (stepAndFinalizeStmt(stmt) == SQLITE_DONE) {
                    _actuatorFeedbackId = actuatorFeedbackId + logNumber;
                }
            }
        }

        int _compassModelId = 0;
        if (compassModelId) {
            bindValues values;
            addValue(values, "heading", log.m_compassHeading);
            addValue(values, "pitch", log.m_compassPitch);
            addValue(values, "roll", log.m_compassRoll);
            addValue(values, "t_timestamp", log.m_timestamp_str);
            if (!prepareStmtInsertError(stmt, "dataLogs_compass", values)) {
                bindValuesToStmt(values, stmt);
                if (stepAndFinalizeStmt(stmt) == SQLITE_DONE) {
                    _actuatorFeedbackId = actuatorFeedbackId + logNumber;
                }
            }
        }

        int _courseCalculationId = 0;
        if (courseCalculationId) {
            bindValues values;
            addValue(values, ":distance_to_waypoint", log.m_distanceToWaypoint);
            addValue(values, ":bearing_to_waypoint", log.m_bearingToWaypoint);
            addValue(values, ":course_to_steer", log.m_courseToSteer);
            addValue(values, ":tack", log.m_tack);
            addValue(values, ":going_starboard", log.m_goingStarboard);
            addValue(values, ":t_timestamp", log.m_timestamp_str);
            if (!prepareStmtInsertError(stmt, "dataLogs_course_calculation", values)) {
                if (stepAndFinalizeStmt(stmt) == SQLITE_DONE) {
                    _courseCalculationId = courseCalculationId + logNumber;
                }
            }
        }

        int _marineSensorsId = 0;
        if (marineSensorsId) {
            bindValues values;
            addValue(values, ":temperature", log.m_temperature);
            addValue(values, ":conductivity", log.m_conductivity);
            addValue(values, ":ph", log.m_ph);
            addValue(values, ":salinity", log.m_salinity);
            addValue(values, ":t_timestamp", log.m_timestamp_str);
            if (!prepareStmtInsertError(stmt, "dataLogs_marine_sensors", values)) {
                if (stepAndFinalizeStmt(stmt) == SQLITE_DONE) {
                    _marineSensorsId = marineSensorsId + logNumber;
                }
            }
        }

        int _vesselStateId = 0;
        if (vesselStateId) {
            bindValues values;
            addValue(values, ":heading", log.m_vesselHeading);
            addValue(values, ":latitude", log.m_vesselLat);
            addValue(values, ":longitude", log.m_vesselLon);
            addValue(values, ":speed", log.m_vesselSpeed);
            addValue(values, ":course", log.m_vesselCourse);
            addValue(values, ":t_timestamp", log.m_timestamp_str);
            if (!prepareStmtInsertError(stmt, "dataLogs_vessel_state", values)) {
                if (stepAndFinalizeStmt(stmt) == SQLITE_DONE) {
                    _vesselStateId = vesselStateId + logNumber;
                }
            }
        }

        int _windStateId = 0;
        if (windStateId) {
            bindValues values;
            addValue(values, ":true_wind_speed", log.m_trueWindSpeed);
            addValue(values, ":true_wind_direction", log.m_trueWindDir);
            addValue(values, ":apparent_wind_speed", log.m_apparentWindSpeed);
            addValue(values, ":apparent_wind_direction", log.m_apparentWindDir);
            addValue(values, ":t_timestamp", log.m_timestamp_str);
            if (!prepareStmtInsertError(stmt, "dataLogs_wind_state", values)) {
                if (stepAndFinalizeStmt(stmt) == SQLITE_DONE) {
                    _windStateId = windStateId + logNumber;
                }
            }
        }

        int _windsensorId = 0;
        if (windsensorId) {
            bindValues values;
            addValue(values, ":direction", log.m_windDir);
            addValue(values, ":speed", log.m_windSpeed);
            addValue(values, ":temperature", log.m_windTemp);
            addValue(values, ":t_timestamp", log.m_timestamp_str);
            if (!prepareStmtInsertError(stmt, "dataLogs_windsensor", values)) {
                if (stepAndFinalizeStmt(stmt) == SQLITE_DONE) {
                    _windsensorId = windsensorId + logNumber;
                }
            }
        }

        int _gpsId = 0;
        if (gpsId) {
        	bindValues values;
            addValue(values, ":has_fix", log.m_gpsHasFix);
            addValue(values, ":online", log.m_gpsOnline);
            addValue(values, ":time", log.m_gpsUnixTime);
            addValue(values, ":latitude", log.m_gpsLat);
            addValue(values, ":longitude", log.m_gpsLon);
            addValue(values, ":speed", log.m_gpsSpeed);
            addValue(values, ":course", log.m_gpsCourse);
            addValue(values, ":satellites_used", log.m_gpsSatellite);
            addValue(values, ":route_started", log.m_routeStarted);
            addValue(values, ":t_timestamp", log.m_timestamp_str);
            if (!prepareStmtInsertError(stmt, "dataLogs_gps", values)) {
	            if (stepAndFinalizeStmt(stmt) == SQLITE_DONE) {
	                _gpsId = gpsId + logNumber;
	            }
            }
        }

        int _currentSensorsId = 0;
        if (currentSensorsId) {
        	bindValues values;
            addValue(values, ":current", log.m_current);
            addValue(values, ":voltage", log.m_voltage);
            addValue(values, ":element", log.m_element);
            addValue(values, ":element_str", log.m_element_str);
            addValue(values, ":t_timestamp", log.m_timestamp_str);

            if (!prepareStmtInsertError(stmt, "dataLogs_current_sensors", values)) {
	            if (stepAndFinalizeStmt(stmt) == SQLITE_DONE) {
	                _currentSensorsId = currentSensorsId + logNumber;
	            }
            }
        }

        bindValues values;
        addValue(values, ":actuator_feedback_id", _actuatorFeedbackId);
        addValue(values, ":compass_id", _compassModelId);
        addValue(values, ":course_calculation_id", _courseCalculationId);
        addValue(values, ":current_sensors_id", _currentSensorsId);
        addValue(values, ":gps_id", _gpsId);
        addValue(values, ":marine_sensors_id", _marineSensorsId);
        addValue(values, ":vessel_state_id", _vesselStateId);
        addValue(values, ":wind_state_id", _windStateId);
        addValue(values, ":windsensor_id", _windsensorId);
        addValue(values, ":current_mission_id", currentMissionId);
        if (!prepareStmtInsertError(stmt, "dataLogs_system", values)) {
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

/*
        if (retCode != SQLITE_OK) {
            Logger::error("%s SQLite COMMIT error: %s (%d)", __PRETTY_FUNCTION__,
                          sqlite3_errstr(retCode), retCode);
            if (m_error != nullptr) {
                Logger::error("%s SQLite DB error: %s", __PRETTY_FUNCTION__, sqlite3_errmsg(db));
                sqlite3_free(m_error);
                m_error = nullptr;
            }
            //            retCode = sqlite3_exec(db, "ROLLBACK TRANSACTION;", NULL, NULL,
            //            &m_error); Logger::info("%s SQLite ROLLBACK returned %s (%d)",
            //            __PRETTY_FUNCTION__,
            //                         sqlite3_errstr(retCode), retCode);
        }
*/
    }
    DBDisconnect();
}

/**
 * Updates a column of a row in a table
 * @param table
 * @param id
 * @param colName
 * @param newValue
 * @return
 */
bool DBHandler::updateTableIdColumnValues(const char *table, int id, bindValues& values) {
	sqlite3_stmt* stmt = nullptr;
	// std::string sql = "UPDATE " + table + " SET " + colName + " = :value";
	if (!prepareStmtUpdateError(stmt, table, id, values)) {
		bindValuesToStmt(values, stmt);
		if (stepAndFinalizeStmt(stmt)) {
			return true;
		}
	}
	Logger::error("%s Error updating %s", __PRETTY_FUNCTION__, table);
	return false;
}

/******************************************************************************
 * DBTransaction() - Atomic transaction, all or nothing gets done
 * @param SQLQuery
 * @return bool
 */
bool DBHandler::DBTransaction(std::string SQLQuery) {
    sqlite3* db = DBConnect();
    m_error = nullptr;

    if (db != nullptr) {
        int resultcode = 0;

        do {
            if (m_error != nullptr) {
                sqlite3_free(m_error);
                m_error = nullptr;
            }
            sqlite3_exec(db, "BEGIN TRANSACTION", nullptr, nullptr, &m_error);
            resultcode = sqlite3_exec(db, SQLQuery.c_str(), nullptr, nullptr, &m_error);
            sqlite3_exec(db, "END TRANSACTION", nullptr, nullptr, &m_error);
        } while (resultcode == SQLITE_BUSY);

        if (m_error != nullptr) {
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

/**
 * Deletes all values from a table
 * @param table name
 */
void DBHandler::clearTable(std::string table) {
	// If no table to delete, doesn't matter
	DBTransaction("DELETE FROM " + table + ";");
}

/**
 * Clears the contents of all datalogs
 */
void DBHandler::clearLogs() {
	std::vector<std::string> datalogTables = getTableNames("dataLogs_%");
	for (const auto &table : datalogTables) {
		clearTable(table);
	}
}

// TODO: Rewrite old
bool DBHandler::updateTableJson(std::string table, std::string data) {
	std::vector<std::string> columns = getColumnInfo("name", table);

	if (columns.empty()) {
		Logger::error("%s Error: no such table %s", __PRETTY_FUNCTION__, table.c_str());
		return false;
	}

	JSON js = JSON::parse(data);

	std::stringstream ss;

	// start at i = 1 to skip the id
	ss << "SET ";
	auto fixedSize = static_cast<int>(js.size());  // Size would sometimes change, added this variable
	for (int i = 1; i < fixedSize; i++) {
		if (fixedSize > 1) {
			ss << columns.at(static_cast<unsigned long>(i)) << " = " << js[columns.at(static_cast<unsigned long>(i))]
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

// TODO: Rewrite old
void DBHandler::updateConfigs(std::string configs) {
	JSON js = JSON::parse(configs);
	if (js.empty()) {
		Logger::error("%s No JSON in \"%s\"", __PRETTY_FUNCTION__, configs);
	}
	std::vector<std::string> tables;

	for (const auto &i : js.items()) {
		tables.push_back(i.key());  // For each table key
	}

	// tables = sailing_config config_buffer etc

	for (const auto &table : tables) {  // for each table in there
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
	std::string DBPrinter;
	std::string tempValue;
	int valuesLimit = 11;  //"Dirty" fix for limiting the amount of values requested from server
	// waypoint entries (amount of fields n = valuesLimit + 1)
	int limitCounter;

	if (not DBTransaction("DELETE FROM current_Mission;")) {
		Logger::error("%s, Error: failed to delete waypoints", __PRETTY_FUNCTION__);
	}

	for (const auto &i : js.items()) {
		// m_logger.info(i.value().dump());

		for (const auto &y : i.value().items()) {
			limitCounter = valuesLimit;
			DBPrinter =
			  "INSERT INTO current_Mission "
			  "(declination,harvested,id,id_mission,is_checkpoint,latitude,longitude,name,radius,"
			  "rankInMission,stay_time) VALUES (";

			for (const auto &z : y.value().items()) {
				// Each individual value
				tempValue = z.value().dump();
				tempValue = tempValue.substr(1, tempValue.size() - 2);
				if (tempValue.empty()) {
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

/*******************************************************************************
 * Utility functions
 ******************************************************************************/

std::string DBHandler::prependString(const std::string &string, const char *const prefix) {
	return std::string(prefix) + string;
}

std::vector<std::string> DBHandler::prependStrings(const std::vector<std::string> &strings, const char *const prefix) {
	std::vector<std::string> result;
	for (const auto &item : strings) {
		result.emplace_back(prependString(item, prefix));
	}
	return std::move(result);
}

// Theese could be put in a separate util lib
// From: https://stackoverflow.com/questions/5288396/c-ostream-out-manipulation/5289170#5289170
// note: separator cannot contain null characters
/***
 * Joins a vector of strings into a combined string with glue as a separator character
 * @param elements
 * @param glue
 * @return
 */
std::string DBHandler::joinStrings(const std::vector<std::string> &elements, const char *const glue) {
	switch (elements.size()) {
		case 0:
			return "";
		case 1:
			return elements[0];
		default:
			std::ostringstream os;
			std::copy(elements.begin(), elements.end()-1, std::ostream_iterator<std::string>(os, glue));
			os << *elements.rbegin();
			return os.str();
	}
}

/***
 * Splits a string into a vector on glue as separating character
 * @param string
 * @param glue
 * @param result
 */
std::vector<std::string> DBHandler::splitStrings(const std::string &string, const char glue) {
	std::vector<std::string> result;
	std::string::const_iterator cur = string.begin();
	std::string::const_iterator beg = string.begin();
	while (cur < string.end()) {
		if (*cur == glue) {
			result.insert(result.end(), std::string(beg, cur));
			beg = ++cur;
		} else {
			cur++;
		}
	}
	result.insert(result.end(), std::string(beg, cur));
	return std::move(result);
}

// NOTE : Marc : change this otherwise it doesn't work
/*	int rows = 0;
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
		return js.dump();*/
// Kill
/*void DBHandler::getDataAsJson(std::string select,
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
}*/


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

/*std::string DBHandler::getLogsAsJSON(bool onlyLatest) {
    JSON js;

    // fetch all datatables starting with "dataLogs_"
    std::vector<std::string> datalogTables = getTableNames("dataLogs_%");
    std::vector<std::vector<std::string>> rows = getRowsAsText()
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
