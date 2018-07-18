#include "DBHandler.h"
#include <cstdio>
#include <cstdlib>
#include <iomanip>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include "../SystemServices/Logger.h"
#include "../SystemServices/Timer.h"
// #include "../SystemServices/Wrapper.h"

// TODO: reusable statements?

/* "Safe" stoi which does not throw exceptions on bad input values */
/*static double safe_stod(const std::string& str, std::size_t* pos = 0, int base = 10) {
    double retvalue = 0;
    try {
        retvalue = std::stod(str, pos);
    } catch (std::invalid_argument& e) {
        Logger::error("%s stod(): invalid argument (%s)", __PRETTY_FUNCTION__, str.c_str());
    } catch (std::out_of_range& e) {
        Logger::error("%s stod(): value out of range (%s)", __PRETTY_FUNCTION__, str.c_str());
    }
    return retvalue;
}*/

std::mutex DBHandler::m_databaseLock;

DBHandler::DBHandler(std::string filePath) : m_filePath(std::move(filePath)) {
    m_latestDataLogId = 0;
}

DBHandler::~DBHandler() {
    DBClose();
    m_databaseLock.unlock();
}

bool DBHandler::initialise() {
    if (!sqlite3_threadsafe()) {
        Logger::warning("%s SQLite is not operating in thread-safe mode!", __PRETTY_FUNCTION__);
    }
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
    if ((retCode == SQLITE_OK) || (retCode == SQLITE_DONE) || (retCode == SQLITE_ROW)) {
        return SQLITE_OK;
    }
    Logger::error("SQLite result code: %s (%d)", sqlite3_errstr(retCode), retCode);
    return retCode;
}

/**
 * sqlite3_step() and sqlite3_finalize() helper function
 * @param stmt
 * @return SQLite error code
 */
int DBHandler::stepAndFinalizeStmt(sqlite3_stmt*& stmt) const {
    int retCode = sqlite3_step(stmt);
    checkRetCode(retCode);
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
int DBHandler::prepareStmtError(sqlite3_stmt*& stmt, const std::string& sql) {
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

int DBHandler::prepareStmtInsertError(sqlite3_stmt*& stmt,
                                      const std::string& table,
                                      std::vector<std::string>& columns) {
    std::string sql = "INSERT INTO " + table + "(" + joinStrings(columns, ",") + ") VALUES(" +
                      joinStrings(prependStrings(":", columns), ",") + ")";
    // sql += ";";  // Not really needed but neat
    return prepareStmtError(stmt, sql);
}

int DBHandler::prepareStmtInsertError(sqlite3_stmt*& stmt,
                                      const std::string& table,
                                      const typedValuePairs& values) {
    std::vector<std::string> names = valueNames(values);
    return prepareStmtInsertError(stmt, table, names);
}

int DBHandler::prepareStmtUpdateError(sqlite3_stmt*& stmt,
                                      const std::string& table,
                                      const int id,
                                      const std::vector<std::string>& columns) {
    std::string sql = "UPDATE " + table + " SET ";
    int i = 0;
    for (const auto& column : columns) {
        if (i++) {
            sql += ",";
        }
        sql += column + "=:" + column;
    }
    sql += " WHERE id = :id";
    // sql += ";";  // Not really needed but neat
    int retCode = prepareStmtError(stmt, sql);
    if (retCode == SQLITE_OK) {
        retCode = bindParam(stmt, ":id", id);
    }
    return retCode;
}

int DBHandler::prepareStmtUpdateError(sqlite3_stmt*& stmt,
                                      const std::string& table,
                                      const int id,
                                      const typedValuePairs& values) {
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
int DBHandler::bindParam(sqlite3_stmt*& stmt, const std::string& name, int value) {
    if (!name.empty()) {
	    int paramIndex = paramNameIndex(stmt, name);
	    if (paramIndex) {
		    return checkRetCode(sqlite3_bind_int(stmt, paramIndex, value));
	    }
    }
	return SQLITE_MISUSE;
}

/**
 * Binds a double to a statement
 * @param stmt		SQLite statement
 * @param name		Parameter name
 * @param value		Value
 * @return
 */
int DBHandler::bindParam(sqlite3_stmt*& stmt, const std::string& name, double value) {
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
int DBHandler::bindParam(sqlite3_stmt*& stmt, const std::string& name, const std::string& text) {
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
int DBHandler::paramNameIndex(sqlite3_stmt *&stmt, std::string name) { // this was const std::string &name
	int paramIndex = 0;
	if (!name.empty()) {
		paramIndex = sqlite3_bind_parameter_index(stmt, name.c_str());
		if (!paramIndex) {
			paramIndex = sqlite3_bind_parameter_index(
			  stmt, prependString(":", name).c_str());  // retry it with an added :
			if (paramIndex) {
				Logger::warning("Added ':' to %s to make it work", name.c_str());
			}
		}
	}
    if (!paramIndex) {
        Logger::error("SQLite null parameter index on \"%s\"!", name.c_str());
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
/*int DBHandler::bindStmtIntsDoublesStrings(
  sqlite3_stmt *&stmt,
  const std::vector<std::pair<const std::string, int>> &ints,
  const std::vector<std::pair<const std::string, double>> &doubles,
  const std::vector<std::pair<const std::string, std::string>> &strings) {
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
}*/

int DBHandler::bindValuesToStmt(const typedValuePairs& values, sqlite3_stmt*& stmt) {
    int firstErrorCode = SQLITE_OK;
    int retCode;  // Uggly but works

    // This could probably be optimized
    for (auto pair : values.ints) {
        retCode = bindParam(stmt, prependString(":", pair.first).c_str(), pair.second);
        if ((firstErrorCode == SQLITE_OK) && (retCode != SQLITE_OK)) {
            firstErrorCode = retCode;
        }
    }
    for (auto pair : values.doubles) {
        retCode = bindParam(stmt, prependString(":", pair.first).c_str(), pair.second);
        if ((firstErrorCode == SQLITE_OK) && (retCode != SQLITE_OK)) {
            firstErrorCode = retCode;
        }
    }
    for (auto pair : values.strings) {
        retCode = bindParam(stmt, prependString(":", pair.first).c_str(), pair.second);
        if ((firstErrorCode == SQLITE_OK) && (retCode != SQLITE_OK)) {
            firstErrorCode = retCode;
        }
    }
    return firstErrorCode;
}

void DBHandler::addValue(typedValuePairs& values, const std::string& name, int value) {
    std::pair<const std::string, int> pair = std::make_pair(name, value);
    values.ints.emplace_back(pair);
}
void DBHandler::addValue(typedValuePairs& values, const std::string& name, double value) {
    std::pair<const std::string, double> pair = std::make_pair(name, value);
    values.doubles.emplace_back(pair);
}
void DBHandler::addValue(typedValuePairs& values, const std::string& name, std::string& string) {
    std::pair<const std::string, std::string> pair = std::make_pair(name, string);
    values.strings.emplace_back(pair);
}

std::vector<std::string> DBHandler::valueNames(const typedValuePairs& values) {
    std::vector<std::string> names;

    // This could probably be optimized
    for (auto pair : values.ints)
        names.push_back(pair.first);
    for (auto pair : values.doubles)
        names.push_back(pair.first);
    for (auto pair : values.strings)
        names.push_back(pair.first);
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
    if (checkRetCode(retCode) == SQLITE_OK) {
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
            checkRetCode(retCode);
        }
    }
    checkRetCode(sqlite3_finalize(stmt));  // Danger, danger!
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
    int retCode;

    retCode = prepareStmtSelectFromStatements(stmt, "name", "sqlite_master",
                                              "WHERE type='table' AND name LIKE :like");
    if (retCode == SQLITE_OK) {
        retCode = bindParam(stmt, ":like", like);
        if (retCode == SQLITE_OK) {
            results = getRowsAsText(stmt);

            for (auto result : results) {
                tableNames.push_back(result[0]);
            }
        }
    }
    return tableNames;
}

/**
 * get id from table returns either max or min id from table
 * @param table
 * @param minmax
 * @return
 */
int DBHandler::getTableId(const std::string& table, ID_MINMAX minmax) {
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
                      table.c_str());
    }
    if (stmt != nullptr)
        sqlite3_finalize(stmt);
    return id;
}

int DBHandler::sqliteTypeFromString(std::string const& typestr) {
    if (typestr == "BOOLEAN")
        return SQLITE_INTEGER;
    if (typestr == "INTEGER")
        return SQLITE_INTEGER;
    if (typestr == "FLOAT")
        return SQLITE_FLOAT;
    if (typestr == "DOUBLE")
        return SQLITE_FLOAT;
    return SQLITE_TEXT;
}

DBHandler::ColumnTypes DBHandler::getTableColumnTypes(const std::string& tableName) {
    std::string sql = "PRAGMA table_info(" + tableName + ")";
    std::vector<std::pair<std::string, int>> result;
    textTableRows rows;
    sqlite3_stmt* stmt;
    if (!prepareStmtError(stmt, sql)) {
        rows = getRowsAsText(stmt);
    }
    for (auto row : rows) {
        std::string name = row[1];
        std::string typestr = row[2];
        result.push_back(std::make_pair(name, sqliteTypeFromString(typestr)));
    }
    return std::move(result);
}

/**
 * Will give you the type string of a named column
 * @param name
 * @param types
 * @return
 */
int DBHandler::columnType(const std::string& name, ColumnTypes& types) {
    for (auto type : types) {
        if (name == type.first)
            return type.second;
    }
    return SQLITE_NULL;
}

// TODO: Rewrite old
// Not sure if this is even needed
std::vector<std::string> DBHandler::getColumnInfo(std::string& info, std::string& table) {
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
    sqlite3_stmt* stmt;
    int retCode =
        prepareStmtSelectFromStatements(stmt, "MIN(id)", "currentMission", "WHERE harvested = 0");
    if (retCode == SQLITE_OK) {
        retCode = sqlite3_step(stmt);
        if (retCode == SQLITE_ROW) {
            sqlite3_column_value(stmt, 0, nextId);
            sqlite3_finalize(stmt);

            std::vector<std::string> columns = {"longitude", "latitude",  "declination",
                                                "radius",    "stay_time", "isCheckpoint"};
            retCode = prepareStmtSelectFromStatements(stmt, joinStrings(columns, ","),
                                                      "currentMission", "WHERE id = :id");
            if (retCode == SQLITE_OK) {
                bindParam(stmt, ":id", nextId);
                retCode = sqlite3_step(stmt);
                if (retCode == SQLITE_ROW) {
                    sqlite3_column_value(stmt, indexOfStringInStrings(columns, "longitude"),
                                         nextLongitude);
                    sqlite3_column_value(stmt, indexOfStringInStrings(columns, "latitude"),
                                         nextLatitude);
                    sqlite3_column_value(stmt, indexOfStringInStrings(columns, "declination"),
                                         nextDeclination);
                    sqlite3_column_value(stmt, indexOfStringInStrings(columns, "radius"),
                                         nextRadius);
                    sqlite3_column_value(stmt, indexOfStringInStrings(columns, "stay_time"),
                                         nextStayTime);
                    sqlite3_column_value(stmt, indexOfStringInStrings(columns, "isCheckpoint"),
                                         isCheckpoint);
                    // selectFromId(nextLongitude, "longitude", "currentMission", nextWayPointId);

	                // Can not use selectFromId() here as we have a more advanced statement
                    selectFrom(prevId, {{std::make_pair("id", nextId)},{},{}}, "MAX(id)", "currentMission", "WHERE id < :id");

                    foundPrev = (prevId != 0);
                    if (foundPrev) {
                        sqlite3_reset(stmt);
                        bindParam(stmt, ":id", prevId);
                        retCode = sqlite3_step(stmt);
                        if (retCode == SQLITE_ROW) {
                            sqlite3_column_value(stmt, indexOfStringInStrings(columns, "longitude"),
                                                 prevLongitude);
                            sqlite3_column_value(stmt, indexOfStringInStrings(columns, "latitude"),
                                                 prevLatitude);
                            sqlite3_column_value(stmt,
                                                 indexOfStringInStrings(columns, "declination"),
                                                 prevDeclination);
                            sqlite3_column_value(stmt, indexOfStringInStrings(columns, "radius"),
                                                 prevRadius);
                            sqlite3_column_value(stmt, indexOfStringInStrings(columns, "stay_time"),
                                                 nextStayTime);
                            sqlite3_column_value(stmt,
                                                 indexOfStringInStrings(columns, "isCheckpoint"),
                                                 isCheckpoint);
                        }
                    }
                }
            }
        }
    }

    if (checkRetCode(retCode) == SQLITE_OK)
        return true;
    Logger::error("%s Errors when retrieving way points from local database", __PRETTY_FUNCTION__);
    return false;
}

// std::vector<std::pair<std::string, std::vector<std::vector<std::string>>>>
DBHandler::textTables DBHandler::getTablesAsText(const std::string& like,
                                                 const std::string& statement) {
    sqlite3_stmt* stmt = nullptr;

    textTableRow tableNames = getTableNames(like);
    textTables tables;

    try {
        // insert all data in these tables as json array
        for (const auto& table : tableNames) {
            prepareStmtSelectFromStatements(stmt, "*", table, statement);
            std::vector<std::vector<std::string>> rows = getRowsAsText(stmt, true);

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
                tables.emplace_back(tableTitle, rows);
            } else {
                Logger::warning("%s, Table %s empty", __PRETTY_FUNCTION__, table.c_str());
            }
        }
    } catch (const char* error) {
        Logger::error("%s, Error gathering data from %s: %s", __PRETTY_FUNCTION__, like.c_str(), error);
    }
    return tables;
}

/**
 * Retreives a table as a vector of pairs tablename - vector of rows (with 0 being a header)
 * converted to JSON
 * @param like
 * @param statement
 * @return
 */
std::string DBHandler::getTablesAsJSON(const std::string& like, const std::string& statement) {
    std::string result;
    JSON js;
    auto tables = getTablesAsText(like, statement);

    try {
        int rowCnt = 0;
        for (auto pair : tables) {
            std::string table = pair.first;
            textTableRows rows = pair.second;

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
        Logger::error("%s, Error JSON-encoding data from %s: %s", __PRETTY_FUNCTION__, like.c_str(), error);
    }
    // tableNames.clear();
    // tables.clear();
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
    std::string result = getTablesAsJSON("currentMission");
    if (result.empty()) {
        Logger::warning("No waypoints in database");
    }
    return result;
}

// TODO: Rewrite old
std::vector<std::string> DBHandler::retrieveFromTable(const std::string& SQLSelect,
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
    int currentSensorsId;
    int gpsId;
    int marineSensorsId;
    int vesselStateId;
    int windStateId;
    int windsensorId;

    int currentMissionId = 0;  // this is not like the others

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
    actuatorFeedbackId  = 1 + getTableId("dataLogs_actuator_feedback");
    compassModelId      = 1 + getTableId("dataLogs_compass");
    courseCalculationId = 1 + getTableId("dataLogs_course_calculation");
    currentSensorsId    = 1 + getTableId("dataLogs_current_sensors");
    gpsId               = 1 + getTableId("dataLogs_gps");
    marineSensorsId     = 1 + getTableId("dataLogs_marine_sensors");
    vesselStateId       = 1 + getTableId("dataLogs_vessel_state");
    windStateId         = 1 + getTableId("dataLogs_wind_state");
    windsensorId        = 1 + getTableId("dataLogs_windsensor");

    selectFrom(currentMissionId, "id_mission", "currentMission", "LIMIT 1");
    // selectFrom(currentWaypointId, "MIN(id)", "currentMission", "WHERE harvested = 0");
    // clang-format on

    for (auto log : logs) {
        std::string sql;
        sqlite3_stmt* stmt = nullptr;

        int _actuatorFeedbackId = 0;
        if (actuatorFeedbackId) {
            typedValuePairs values = {{},{},{}};
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
	        typedValuePairs values = {{},{},{}};
            addValue(values, "heading", log.m_compassHeading);
            addValue(values, "pitch", log.m_compassPitch);
            addValue(values, "roll", log.m_compassRoll);
            addValue(values, "t_timestamp", log.m_timestamp_str);
            if (!prepareStmtInsertError(stmt, "dataLogs_compass", values)) {
                bindValuesToStmt(values, stmt);
                if (stepAndFinalizeStmt(stmt) == SQLITE_DONE) {
                    _compassModelId = compassModelId + logNumber;
                }
            }
        }

        int _courseCalculationId = 0;
        if (courseCalculationId) {
	        typedValuePairs values = {{},{},{}};
            addValue(values, "distance_to_waypoint", log.m_distanceToWaypoint);
            addValue(values, "bearing_to_waypoint", log.m_bearingToWaypoint);
            addValue(values, "course_to_steer", log.m_courseToSteer);
            addValue(values, "tack", log.m_tack);
            addValue(values, "going_starboard", log.m_goingStarboard);
            addValue(values, "t_timestamp", log.m_timestamp_str);
            if (!prepareStmtInsertError(stmt, "dataLogs_course_calculation", values)) {
                bindValuesToStmt(values, stmt);
                if (stepAndFinalizeStmt(stmt) == SQLITE_DONE) {
                    _courseCalculationId = courseCalculationId + logNumber;
                }
            }
        }

        int _marineSensorsId = 0;
        if (marineSensorsId) {
	        typedValuePairs values = {{},{},{}};
            addValue(values, "temperature", log.m_temperature);
            addValue(values, "conductivity", log.m_conductivity);
            addValue(values, "ph", log.m_ph);
            addValue(values, "salinity", log.m_salinity);
            addValue(values, "t_timestamp", log.m_timestamp_str);
            if (!prepareStmtInsertError(stmt, "dataLogs_marine_sensors", values)) {
                bindValuesToStmt(values, stmt);
                if (stepAndFinalizeStmt(stmt) == SQLITE_DONE) {
                    _marineSensorsId = marineSensorsId + logNumber;
                }
            }
        }

        int _vesselStateId = 0;
        if (vesselStateId) {
	        typedValuePairs values = {{},{},{}};
            addValue(values, "heading", log.m_vesselHeading);
            addValue(values, "latitude", log.m_vesselLat);
            addValue(values, "longitude", log.m_vesselLon);
            addValue(values, "speed", log.m_vesselSpeed);
            addValue(values, "course", log.m_vesselCourse);
            addValue(values, "t_timestamp", log.m_timestamp_str);
            if (!prepareStmtInsertError(stmt, "dataLogs_vessel_state", values)) {
                bindValuesToStmt(values, stmt);
                if (stepAndFinalizeStmt(stmt) == SQLITE_DONE) {
                    _vesselStateId = vesselStateId + logNumber;
                }
            }
        }

        int _windStateId = 0;
        if (windStateId) {
	        typedValuePairs values = {{},{},{}};
            addValue(values, "true_wind_speed", log.m_trueWindSpeed);
            addValue(values, "true_wind_direction", log.m_trueWindDir);
            addValue(values, "apparent_wind_speed", log.m_apparentWindSpeed);
            addValue(values, "apparent_wind_direction", log.m_apparentWindDir);
            addValue(values, "t_timestamp", log.m_timestamp_str);
            if (!prepareStmtInsertError(stmt, "dataLogs_wind_state", values)) {
                bindValuesToStmt(values, stmt);
                if (stepAndFinalizeStmt(stmt) == SQLITE_DONE) {
                    _windStateId = windStateId + logNumber;
                }
            }
        }

        int _windsensorId = 0;
        if (windsensorId) {
	        typedValuePairs values = {{},{},{}};
            addValue(values, "direction", log.m_windDir);
            addValue(values, "speed", log.m_windSpeed);
            addValue(values, "temperature", log.m_windTemp);
            addValue(values, "t_timestamp", log.m_timestamp_str);
            if (!prepareStmtInsertError(stmt, "dataLogs_windsensor", values)) {
                bindValuesToStmt(values, stmt);
                if (stepAndFinalizeStmt(stmt) == SQLITE_DONE) {
                    _windsensorId = windsensorId + logNumber;
                }
            }
        }

        int _gpsId = 0;
        if (gpsId) {
	        typedValuePairs values = {{},{},{}};
            addValue(values, "has_fix", log.m_gpsHasFix);
            addValue(values, "online", log.m_gpsOnline);
            addValue(values, "time", log.m_gpsUnixTime);
            addValue(values, "latitude", log.m_gpsLat);
            addValue(values, "longitude", log.m_gpsLon);
            addValue(values, "speed", log.m_gpsSpeed);
            addValue(values, "course", log.m_gpsCourse);
            addValue(values, "satellites_used", log.m_gpsSatellite);
            addValue(values, "route_started", log.m_routeStarted);
            addValue(values, "t_timestamp", log.m_timestamp_str);
            if (!prepareStmtInsertError(stmt, "dataLogs_gps", values)) {
                bindValuesToStmt(values, stmt);
                if (stepAndFinalizeStmt(stmt) == SQLITE_DONE) {
                    _gpsId = gpsId + logNumber;
                }
            }
        }

        int _currentSensorsId = 0;
        if (currentSensorsId) {
	        typedValuePairs values = {{},{},{}};
            addValue(values, "current", log.m_current);
            addValue(values, "voltage", log.m_voltage);
            addValue(values, "element", log.m_element);
            addValue(values, "element_str", log.m_element_str);
            addValue(values, "t_timestamp", log.m_timestamp_str);
            if (!prepareStmtInsertError(stmt, "dataLogs_current_sensors", values)) {
                bindValuesToStmt(values, stmt);
                if (stepAndFinalizeStmt(stmt) == SQLITE_DONE) {
                    _currentSensorsId = currentSensorsId + logNumber;
                }
            }
        }

        typedValuePairs values;
        values.ints.clear();
        values.doubles.clear();
        values.strings.clear();
        addValue(values, "actuator_feedback_id", _actuatorFeedbackId);
        addValue(values, "compass_id", _compassModelId);
        addValue(values, "course_calculation_id", _courseCalculationId);
        addValue(values, "current_sensors_id", _currentSensorsId);
        addValue(values, "gps_id", _gpsId);
        addValue(values, "marine_sensors_id", _marineSensorsId);
        addValue(values, "vessel_state_id", _vesselStateId);
        addValue(values, "wind_state_id", _windStateId);
        addValue(values, "windsensor_id", _windsensorId);
        addValue(values, "current_mission_id", currentMissionId);
        if (!prepareStmtInsertError(stmt, "dataLogs_system", values)) {
            bindValuesToStmt(values, stmt);
            if (stepAndFinalizeStmt(stmt) == SQLITE_DONE) {
                m_latestDataLogId = getTableId("dataLogs_system");
            } else {
                m_latestDataLogId = 0;
                Logger::error("%s Error, failed to create system log index!", __PRETTY_FUNCTION__);
            }
        }
        logNumber++;
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
bool DBHandler::updateTableRow(const std::string& table, int id, const typedValuePairs& values) {
    sqlite3_stmt* stmt = nullptr;
    if (prepareStmtUpdateError(stmt, table, id, values) == SQLITE_OK) {
        if (bindValuesToStmt(values, stmt) == SQLITE_OK) {
            if (checkRetCode(stepAndFinalizeStmt(stmt)) == SQLITE_OK) {
                return true;
            }
        }
    }
    Logger::error("%s Error updating %s", __PRETTY_FUNCTION__, table.c_str());
    return false;
}

bool DBHandler::insertTableRow(const std::string& tableName, const typedValuePairs& values) {
    sqlite3_stmt* stmt = nullptr;
    if (prepareStmtInsertError(stmt, tableName, values) == SQLITE_OK) {
        if (bindValuesToStmt(values, stmt) == SQLITE_OK) {
            if (checkRetCode(stepAndFinalizeStmt(stmt)) == SQLITE_OK) {
                return true;
            }
        }
    }
    Logger::error("%s Error updating %s", __PRETTY_FUNCTION__, tableName.c_str());
    return false;
}

int DBHandler::insertTableRowsErrors(const std::string& tableName, const tableRows& rows) {
    sqlite3_stmt* stmt = nullptr;
    int retCode;
    int errors = 0;
    if (!rows.empty()) {
        auto row = rows.begin();
        if (prepareStmtInsertError(stmt, tableName, *row) == SQLITE_OK) {
            while (row != rows.end()) {
                retCode = bindValuesToStmt(*row, stmt);
                if (retCode == SQLITE_OK) {
                    retCode = checkRetCode(sqlite3_step(stmt));
                } else {
                    errors++;
                }
                sqlite3_reset(stmt);
                row++;
            }
        } else {
            errors++;
        }
        sqlite3_finalize(stmt);
        if (errors == 0) {
            return false;
        }
    }
    Logger::error("%s %d errors updating %s", __PRETTY_FUNCTION__, errors, tableName.c_str());
    return true;
}

bool DBHandler::transactionalReplaceTable(const std::string& tableName, const tableRows& rows) {
    sqlite3* db = DBConnect();
    int retCode;
    std::string sql = "BEGIN TRANSACTION";
    retCode = sqlite3_exec(db, sql.c_str(), nullptr, nullptr, &m_error);
    if (retCode == SQLITE_OK) {
        sql = "DELETE FROM " + std::string(tableName);
        retCode = sqlite3_exec(db, sql.c_str(), nullptr, nullptr, &m_error);
        if (checkRetCode(retCode) != SQLITE_OK) {
            Logger::error("%s %s (%d) deleting data in table %s", __PRETTY_FUNCTION__,
                          sqlite3_errstr(retCode), retCode, tableName.c_str());
        }
    }
    if (!insertTableRowsErrors(tableName, rows)) {
        sql = "END TRANSACTION";
        retCode = sqlite3_exec(db, sql.c_str(), nullptr, nullptr, &m_error);
    }
    if (checkRetCode(retCode) == SQLITE_OK) {
        return true;
    }
    Logger::error("%s %s (%d) when replacing data in table %s", __PRETTY_FUNCTION__,
                  sqlite3_errstr(retCode), retCode, tableName.c_str());
    retCode = sqlite3_exec(db, "ROLLBACK TRANSACTION;", nullptr, nullptr, &m_error);
    if (retCode != SQLITE_OK) {
        Logger::info("%s SQLite ROLLBACK returned %s (%d)", __PRETTY_FUNCTION__,
                     sqlite3_errstr(retCode), retCode);
    }
    return false;
}

bool DBHandler::transactionalReplaceTable(const std::string& tableName, const textTableRows& rows) {
    tableRows values;
    std::vector<std::pair<std::string, int>> columnTypes = getTableColumnTypes(tableName);
    valuesFromTextRows(values, rows, columnTypes);
    return transactionalReplaceTable(tableName, values);
}

bool DBHandler::transactionalReplaceTable(const textTable& table) {
    return transactionalReplaceTable(table.first.c_str(), table.second);
}

bool DBHandler::replaceTables(const textTables& tables) {
    int errors = 0;
    for (auto table : tables) {
        if (!transactionalReplaceTable(table)) {
            errors++;
        }
    }
    // TODO: Log errors
    return (errors == 0);
}

void DBHandler::valuesFromTextRows(tableRows& values,
                                   const textTableRows& textRows,
                                   const ColumnTypes& types) {
    auto row = textRows.begin();
    textTableRow columnNames = *row;  // First row is column names

    // detect types here, look at column info? For now we cheat and treat it as text
    while (++row != textRows.end()) {
        auto name = columnNames.begin();
        auto value = (*row).begin();
        typedValuePairs rowValues;
        while ((name != columnNames.end()) && (value != (*row).end())) {
            // Cheat by telling SQLite we have all strings and let it sort it out
            rowValues.strings.emplace_back(std::make_pair((*name).c_str(), *value));
            name++;
            value++;
        }
        values.emplace_back(rowValues);
    }
}

/******************************************************************************
 * DBTransaction() - Atomic transaction, all or nothing gets done
 * @param SQLQuery
 * @return bool
 */
bool DBHandler::DBTransaction(const std::string& SQLQuery) {
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
void DBHandler::clearTable(const std::string& table) {
    // If no table to delete, doesn't matter
    DBTransaction("DELETE FROM " + table + ";");
}

/**
 * Clears the contents of all datalogs
 */
void DBHandler::clearLogs() {
    std::vector<std::string> datalogTables = getTableNames("dataLogs_%");
    for (const auto& table : datalogTables) {
        clearTable(table);
    }
}

/*<std::vector<std::pair<std::string, std::vector<std::string>>> DBHandler::JSONtoTables(std::string
data) { JSON js = JSON::parse(data); for (auto element : js) { std::string name = element
    }
}*/

/* // TODO: Rewrite old
bool DBHandler::updateTableJson(const std::string& table, const std::string& data) {
    std::vector<std::string> columns = getColumnInfo("name", table);

    if (columns.empty()) {
        Logger::error("%s Error: no such table %s", __PRETTY_FUNCTION__, table.c_str());
        return false;
    }

    JSON js = JSON::parse(data);
    std::stringstream ss;

    // start at i = 1 to skip the id
    ss << "SET ";
    auto fixedSize =
        static_cast<int>(js.size());  // Size would sometimes change, added this variable
    for (int i = 1; i < fixedSize; i++) {
        if (fixedSize > 1) {
            ss << columns.at(static_cast<unsigned long>(i)) << " = "
               << js[columns.at(static_cast<unsigned long>(i))]
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
}*/

bool DBHandler::JSONAsTables(const std::string& string, textTables& tables) {
    JSON js = JSON::parse(string);
    for (const auto& jstable : js.items()) {
        std::string tableName = jstable.key();
        textTableRows table;
        for (const auto& jsrow : jstable.value().items()) {
            textTableRow row;
            for (const auto& jscolumn : jsrow.value().items()) {
                if (jscolumn.value().is_string()) {
                    std::string content = jscolumn.value().dump();
                    content.erase(remove(content.begin(), content.end(), '\"'),
                                  content.end());  // Remove all double-quote characters

                    row.emplace_back(content);
                } else {
                    Logger::warning("%s Problems parsing contents of \"%s\"", __PRETTY_FUNCTION__,
                                    tableName.c_str());
                    return false;
                }
            }
            table.emplace_back(row);
        }
        tables.emplace_back(std::make_pair(tableName, table));
    }
    return true;
}

void DBHandler::updateConfigs(const std::string& configsJSON) {
    textTables tables;
    if (!JSONAsTables(configsJSON, tables)) {
        Logger::error("%s Unable to parse config JSON \"%s\"", __PRETTY_FUNCTION__,
                      configsJSON.c_str());
    } else {
        if (!replaceTables(tables)) {
            Logger::error("%s failed to get new configs", __PRETTY_FUNCTION__);
        }
    }

    /*JSON js = JSON::parse(configsJSON);
    if (js.empty()) {
        Logger::error("%s No JSON in \"%s\"", __PRETTY_FUNCTION__, configsJSON);
    }
    std::vector<std::string> tables;

    for (const auto& i : js.items()) {
        tables.push_back(i.key());  // For each table key
    }

    // tables = sailing_config config_buffer etc

    for (const auto& table : tables) {  // for each table in there
        if (js[table] != NULL) {
            updateTableJson(table, js[table].dump());  // eg updatetablejson("sailing_config",
                                                       // configs['sailing_config'] as json)
        }
    }*/
}

/**
 * Processes waypoint data as JSON and replaces current DB contents
 * @param wayPointsJSON
 * @return
 */
bool DBHandler::receiveWayPoints(const std::string& wayPointsJSON) {
    textTables tables;
    if (JSONAsTables(wayPointsJSON, tables)) {
        if (replaceTables(tables)) {
            return true;
        }
        Logger::error("%s failed to get new waypoints", __PRETTY_FUNCTION__);
    } else {
        Logger::error("%s Unable to parse waypoint JSON \"%s\"", __PRETTY_FUNCTION__,
                      wayPointsJSON.c_str());
    }
    return false;
}

/*******************************************************************************
 * Utility functions
 ******************************************************************************/

std::string DBHandler::prependString(const std::string& prefix, const std::string& str) {
    std::string ret = prefix + str;
    return std::move(ret);
}

std::vector<std::string> DBHandler::prependStrings(const std::string& prefix,
                                                   const std::vector<std::string>& strings) {
    std::vector<std::string> result;
    for (const auto& item : strings) {
        result.emplace_back(prependString(prefix, item));
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
std::string DBHandler::joinStrings(const std::vector<std::string>& elements,
                                   const std::string& glue) {
    switch (elements.size()) {
        case 0:
            return "";
        case 1:
            return elements[0];
        default:
            std::ostringstream os;
            std::copy(elements.begin(), elements.end() - 1,
                      std::ostream_iterator<std::string>(os, glue.c_str()));
            os << *elements.rbegin();
            return std::move(os.str());
    }
}

/***
 * Splits a string into a vector on glue as separating character
 * @param string
 * @param glue
 * @param result
 */
std::vector<std::string> DBHandler::splitStrings(const std::string& string, const char glue) {
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

int DBHandler::indexOfStringInStrings(const std::vector<std::string>& haystack,
                                      const std::string& needle) {
    int i = 0;
    for (const auto& string : haystack) {
        if (string == needle) {
            return i;
        }
        i++;
    }
    return -1;
}

/*    JSON js = JSON::parse(wayPoints);
    if (js.empty()) {
        Logger::error("%s No JSON in \"%s\"", __PRETTY_FUNCTION__, wayPoints.c_str());
        return false;
    }


    std::string DBPrinter;
    std::string tempValue;
    int valuesLimit = 11;  //"Dirty" fix for limiting the amount of values requested from server
    // waypoint entries (amount of fields n = valuesLimit + 1)
    int limitCounter;

    if (not DBTransaction("DELETE FROM currentMission;")) {
        Logger::error("%s, Error: failed to delete waypoints", __PRETTY_FUNCTION__);
    }

    for (const auto& i : js.items()) {
        // m_logger.info(i.value().dump());

        for (const auto& y : i.value().items()) {
            limitCounter = valuesLimit;
            DBPrinter =
                "INSERT INTO currentMission "
                "(declination,harvested,id,id_mission,is_checkpoint,latitude,longitude,name,radius,"
                "rankInMission,stay_time) VALUES (";

            for (const auto& z : y.value().items()) {
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
        std::string updateHarvested = "UPDATE currentMission SET harvested = 1 WHERE id < ";
        updateHarvested += m_currentWaypointId + ";";

        if (not DBTransaction(updateHarvested)) {
            Logger::error("%s, Error: failed to harvest waypoints", __PRETTY_FUNCTION__);
            return false;
        }
    }
    return true;*/

// NOTE : Marc : change this otherwise it doesn't work
/*	int rows = 0;
    JSON js;
    std::string wp = "waypoint_";

    rows = getRows("currentMission");
    // std::cout << "rows current mission " << rows << std::endl;
    if (rows > 0) {
        for (auto i = 1; i <= rows; ++i) {
            // getDataAsJson("id,is_checkpoint,latitude,longitude,declination,radius,stay_time",
            // "currentMission", wp + std::to_string(i), std::to_string(i),json, true);
            getDataAsJson("id,is_checkpoint,latitude,longitude,declination,radius,stay_time",
                          "currentMission", wp + std::to_string(i), std::to_string(i), js, true);
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
