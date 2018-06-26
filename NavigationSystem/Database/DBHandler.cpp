#include "DBHandler.h"
#include <cstdio>
#include <cstdlib>
#include <iomanip>
#include <string>
#include <thread>
#include "../SystemServices/Timer.h"
#include "../SystemServices/Wrapper.h"


std::mutex DBHandler::m_databaseLock;

DBHandler::DBHandler(std::string filePath) : m_filePath(std::move(filePath)) {
    m_latestDataLogId = 0;
}

DBHandler::~DBHandler() {
    DBClose();
    m_databaseLock.unlock();
}

bool DBHandler::initialise() {
    // Basically just try opening the DB and close it afterwards
    sqlite3* connection = DBConnect();
    DBDisconnect();
    return connection;  // NULL would mean we got no connection
}

void DBHandler::getDataAsJson(std::string select,
                              std::string table,
                              std::string key,
                              std::string id,
                              Json& js,
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

    Json jsonEntry;
    for (int i = 0; i < rows; i++) {
        for (unsigned int j = 0; j < columnNames.size(); j++) {
            int index = j + (columns * i);
            jsonEntry[columnNames.at(j)] = values.at(index);
        }

        if (useArray) {
            if (!js[key].is_array()) {
                js[key] = Json::array({});
            }
            js[key].push_back(jsonEntry);
        } else {
            js[key] = jsonEntry;
        }
    }

    values.clear();
    columnNames.clear();
}

int DBHandler::checkResultCode(const int resultCode) const {
    if (!((resultCode == SQLITE_OK) || (resultCode == SQLITE_DONE))) {
        Logger::error("SQLite result code: %s (%d)", sqlite3_errstr(resultCode), resultCode);
    }
    return resultCode;
}

int DBHandler::prepareStmt(sqlite3* db, std::string sql, sqlite3_stmt* stmt) {
    int resultCode = sqlite3_prepare_v2(db, sql.c_str(), (int) sql.size(), &stmt, NULL);
    return resultCode;
}

// Might be suitable for templates
int DBHandler::bindParam(sqlite3_stmt* stmt, int param, int value) {
    int resultCode = sqlite3_bind_int(stmt, param, value);
    if (checkResultCode(resultCode)) {
        Logger::error("%s parameter %s=%d", __PRETTY_FUNCTION__,
                      sqlite3_bind_parameter_name(stmt, param), param, value);
    }
    return resultCode;
}

int DBHandler::bindParam(sqlite3_stmt* stmt, int param, double value) {
    int resultCode = 0;

    if (param) {
        resultCode = sqlite3_bind_double(stmt, param, value);

        if (checkResultCode(resultCode)) {
            Logger::error("%s parameter %s (%d) =%f", __PRETTY_FUNCTION__,
                          sqlite3_bind_parameter_name(stmt, param), param, value);
        }
    } else {
        Logger::error("%s tried to bind null parameter!", __PRETTY_FUNCTION__);
        return SQLITE_ERROR;
    }
    return resultCode;
}

void DBHandler::insertDataLogs(std::vector<LogItem>& logs) {
    std::stringstream actuatorFeedbackValues;
    std::stringstream compassModelValues;
    std::stringstream courseCalculationValues;
    std::stringstream currentSensorsValues;
    std::stringstream gpsValues;
    std::stringstream marineSensorsValues;
    std::stringstream vesselStateValues;
    std::stringstream windStateValues;
    std::stringstream windsensorValues;
    std::stringstream systemValues;
    std::stringstream ss;
    int actuatorFeedbackId = 0;
    int compassModelId = 0;
    int courseCalculationId = 0;
    int currentMissionId = 0;
    int currentSensorsId = 0;
    int gpsId = 0;
    int marineSensorsId = 0;
    int vesselStateId = 0;
    int windStateId = 0;
    int windsensorId = 0;
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

        if (actuatorFeedbackId) {
            /*
                        actuatorFeedbackValues.str("");
                        actuatorFeedbackValues << std::setprecision(10) << log.m_rudderPosition <<
               ", "
                                               << log.m_wingsailPosition << ", " <<
               log.m_radioControllerOn
                                               << ", " << log.m_windVaneAngle << ",'"
                                               << log.m_timestamp_str.c_str();

                        ss << "INSERT INTO "
                           << "dataLogs_actuator_feedback"
                           << " VALUES(NULL, " << actuatorFeedbackValues.str() << "'); \n";
            */

            // TODO: "Syntax ok but nothing in the DB

            sqlite3_exec(db, "BEGIN TRANSACTION;", NULL, NULL, &m_error);
            std::string sql;
            sqlite3_stmt* stmt = NULL;
            int resultCode;

            sql =
                "INSERT INTO dataLogs_actuator_feedback(id, rudder_position, wingsail_position, rc_on, "
                "wind_vane_angle, t_timestamp)"
                "VALUES(NULL, :rudder_position, :wingsail_position, :rc_on, :wind_vane_angle, "
                ":t_timestamp);";

            // resultCode = sqlite3_prepare_v2(db, sql.c_str(), (int)sql.size(), &stmt, NULL);
            resultCode = prepareStmt(db, sql, stmt);
            if (resultCode == SQLITE_OK) {
                // Logger::info("DEBUG: %d parameters : %s", sqlite3_bind_parameter_count(stmt),
                //             sql.c_str());
                bindParam(stmt, sqlite3_bind_parameter_index(stmt, ":rudder_position"), log.m_rudderPosition);
                bindParam(stmt, sqlite3_bind_parameter_index(stmt, ":wingsail_position"), log.m_wingsailPosition);
                bindParam(stmt, sqlite3_bind_parameter_index(stmt, ":rc_on"), log.m_radioControllerOn);
                bindParam(stmt, sqlite3_bind_parameter_index(stmt, ":wind_vane_angle"), log.m_windVaneAngle);
                sqlite3_bind_text(stmt, sqlite3_bind_parameter_index(stmt, ":t_timestamp"),
                          log.m_timestamp_str.c_str(),
                          log.m_timestamp_str.size(),  // TODO: No _ in timestamp!
                          SQLITE_STATIC);
                checkResultCode(sqlite3_step(stmt));
                checkResultCode(sqlite3_finalize(stmt));
            } else {
                Logger::error("%s statement prepare error: %s (on \"%s\")",
                              __PRETTY_FUNCTION__, sqlite3_errstr(resultCode), sql);
            }

            resultCode = sqlite3_exec(db, "COMMIT;", NULL, NULL, &m_error);
            // resultCode = sqlite3_exec(db, "END TRANSACTION;", NULL, NULL, &m_error);
            if (resultCode != SQLITE_OK) {
                if (m_error != NULL) {
                    Logger::error("%s SQLITE commit error code  %s", __PRETTY_FUNCTION__,
                                  sqlite3_errmsg(db));
                    sqlite3_free(m_error);
                    m_error = NULL;
                }
            }
        }

        compassModelValues.str("");
        compassModelValues << std::setprecision(10) << log.m_compassHeading << ", "
                           << log.m_compassPitch << ", " << log.m_compassRoll << ",'"
                           << log.m_timestamp_str.c_str();

        ss << "INSERT INTO "
           << "dataLogs_compass"
           << " VALUES(NULL, " << compassModelValues.str() << "'); \n";

        courseCalculationValues.str("");
        courseCalculationValues << std::setprecision(10) << log.m_distanceToWaypoint << ", "
                                << log.m_bearingToWaypoint << ", " << log.m_courseToSteer << ", "
                                << log.m_tack << ", " << log.m_goingStarboard << ",'"
                                << log.m_timestamp_str.c_str();

        ss << "INSERT INTO "
           << "dataLogs_course_calculation"
           << " VALUES(NULL, " << courseCalculationValues.str() << "'); \n";

        marineSensorsValues.str("");
        marineSensorsValues << std::setprecision(10) << log.m_temperature << ", "
                            << log.m_conductivity << ", " << log.m_ph << ", " << log.m_salinity
                            << ",'" << log.m_timestamp_str.c_str();

        ss << "INSERT INTO "
           << "dataLogs_marine_sensors"
           << " VALUES(NULL, " << marineSensorsValues.str() << "'); \n";

        vesselStateValues.str("");
        vesselStateValues << std::setprecision(10) << log.m_vesselHeading << ", " << log.m_vesselLat
                          << ", " << log.m_vesselLon << ", " << log.m_vesselSpeed << ", "
                          << log.m_vesselCourse << ",'" << log.m_timestamp_str.c_str();

        ss << "INSERT INTO "
           << "dataLogs_vessel_state"
           << " VALUES(NULL, " << vesselStateValues.str() << "'); \n";

        windStateValues.str("");
        windStateValues << std::setprecision(10) << log.m_trueWindSpeed << ", " << log.m_trueWindDir
                        << ", " << log.m_apparentWindSpeed << ", " << log.m_apparentWindDir << ",'"
                        << log.m_timestamp_str.c_str();

        ss << "INSERT INTO "
           << "dataLogs_wind_state"
           << " VALUES(NULL, " << windStateValues.str() << "'); \n";

        windsensorValues.str("");
        windsensorValues << std::setprecision(10) << log.m_windDir << ", " << log.m_windSpeed
                         << ", " << log.m_windTemp << ",'" << log.m_timestamp_str.c_str();

        ss << "INSERT INTO "
           << "dataLogs_windsensor"
           << " VALUES(NULL, " << windsensorValues.str() << "'); \n";

        // NEW FEATURE, CAREFUL WITH THE APOSTROPHE (added in dbloggernode.h for now, maybe add it
        // in getelementstr)
        currentSensorsValues.str("");
        currentSensorsValues << std::setprecision(10) << log.m_current << ", " << log.m_voltage
                             << ", " << log.m_element << ", '"  // HERE (moved to the m_element_str)
                             << log.m_element_str.c_str()
                             << "', '"  // AND HERE (moved to the m_element_str)
                             << log.m_timestamp_str.c_str();

        ss << "INSERT INTO "
           << "dataLogs_current_sensors"
           << " VALUES(NULL, " << currentSensorsValues.str() << "'); \n";

        gpsValues.str("");
        gpsValues << std::setprecision(10) << log.m_gpsHasFix << ", " << log.m_gpsOnline << ",'"
                  << log.m_timestamp_str.c_str() << "', " << log.m_gpsLat << ", " << log.m_gpsLon
                  << ", " << log.m_gpsSpeed << ", " << log.m_gpsCourse << ", " << log.m_gpsSatellite
                  << ", " << log.m_routeStarted << ",'" << log.m_timestamp_str.c_str();

        ss << "INSERT INTO "
           << "dataLogs_gps"
           << " VALUES(NULL, " << gpsValues.str() << "'); \n";

        // Logger::info("Current sensors database insert command: %s \n", ss);
        // std::cout << "Current sensors database insert command: " << currentSensorsValues.str() <<
        // std::endl; std::cout << "Full insert command line: " << ss.str() << std::endl;

        systemValues.str("");
        systemValues << std::setprecision(10) << actuatorFeedbackId + logNumber << ", "
                     << compassModelId + logNumber << ", " << courseCalculationId + logNumber
                     << ", " << currentSensorsId + logNumber << ", " << gpsId + logNumber << ", "
                     << marineSensorsId + logNumber << ", " << vesselStateId + logNumber << ", "
                     << windStateId + logNumber << ", " << windsensorId + logNumber << ", "
                     << currentMissionId;

        ss << "INSERT INTO "
           << "dataLogs_system"
           << " VALUES(NULL, " << systemValues.str() << "); \n";

        if (DBTransaction(ss.str())) {
            m_latestDataLogId = getTableId("dataLogs_system");
        } else {
            m_latestDataLogId = 0;
            Logger::error("%s Error, failed to insert log Request: %s", __PRETTY_FUNCTION__,
                          ss.str().c_str());
        }
    }

    DBDisconnect();
}

bool DBHandler::updateTableJson(std::string table, std::string data) {
    std::vector<std::string> columns = getColumnInfo("name", table);

    if (columns.size() <= 0) {
        Logger::error("%s Error: no such table %s", __PRETTY_FUNCTION__, table.c_str());
        return false;
    }

    Json js = Json::parse(data);

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

bool DBHandler::updateTableJsonObject(std::string table, Json data) {
    // m_logger.info(" updateTableJson:\n"+data);
    std::vector<std::string> columns = getColumnInfo("name", table);

    if (columns.size() <= 0) {
        Logger::error("%s Error: no such table %s", __PRETTY_FUNCTION__, table.c_str());
        return false;
    }

    // Json json = data;

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
}

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

std::string DBHandler::retrieveCell(std::string table, std::string id, std::string column) {
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
}

void DBHandler::updateConfigs(std::string configs) {
    Json js = Json::parse(configs);
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

bool DBHandler::updateWaypoints(std::string waypoints) {
    Json js = Json::parse(waypoints);
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

int DBHandler::retrieveCellAsInt(std::string table, std::string id, std::string column) {
    std::string data = retrieveCell(table, id, column);
    if (data.size() > 0) {
        return strtol(data.c_str(), NULL, 10);
    } else {
        Logger::error("%s, Error: No data in cell ", __PRETTY_FUNCTION__);
        return 0;
    }
}

double DBHandler::retrieveCellAsDouble(std::string table, std::string id, std::string column) {
    std::string data = retrieveCell(table, id, column);
    if (data.size() > 0) {
        return strtod(data.c_str(), NULL);
    } else {
        Logger::error("%s, Error: No data in cell ", __PRETTY_FUNCTION__);
        return 0;
    }
}

void DBHandler::clearTable(std::string table) {
    // If no table to delete, doesn't matter
    DBTransaction("DELETE FROM " + table + ";");
}

int DBHandler::getRows(std::string table) {
    int columns, rows;
    try {
        retrieveFromTable("SELECT * FROM " + table + ";", rows, columns);
    } catch (const char* error) {
        return 0;
    }
    return rows;
}

std::string DBHandler::getLogs(bool onlyLatest) {
    Json js;

    // fetch all datatables starting with "dataLogs_"
    std::vector<std::string> datalogTables = getTableNames("dataLogs_%");

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
}

void DBHandler::clearLogs() {
    std::vector<std::string> datalogTables = getTableNames("dataLogs_%");

    for (auto table : datalogTables) {
        clearTable(table);
    }
}

void DBHandler::deleteRow(std::string table, std::string id) {
    DBTransaction("DELETE FROM " + table + " WHERE id = " + id + ";");
}

bool DBHandler::insert(std::string table, std::string fields, std::string values) {
    if (not DBTransaction("INSERT INTO " + table + "(" + fields + ") VALUES(" + values + ");")) {
        Logger::error("%s, Failed to insert into table", __PRETTY_FUNCTION__);
        return false;
    }
    return true;
}

std::string DBHandler::getWaypoints() {  // NOTE : Marc : change this otherwise it doesn't work
    int rows = 0;
    Json js;
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

// TODO there is a segfault in here somewhere
int DBHandler::getTableId(std::string table, ID_MINMAX minmax) {
    int rows, columns;
    std::vector<std::string> results;

    try {
        if (minmax == MAX_ID) {
            results = retrieveFromTable("SELECT MAX(id) FROM " + table + ";", rows, columns);
        } else {
            results = retrieveFromTable("SELECT MIN(id) FROM " + table + ";", rows, columns);
        }
    } catch (const char* error) {
        Logger::error("%s Error when determining min/max id from %s", __PRETTY_FUNCTION__, table);
        return 0;
    }
    if (results.empty()) {
        Logger::warning("%s Empty result when determining min/max id from %s", __PRETTY_FUNCTION__,
                        table);
        return 0;
    }
    return safe_stoi(results[1], NULL, 10);
}

/******************************************************************************
 * private helpers
 *****************************************************************************/

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

void DBHandler::DBDisconnect() {
    // Logger::info("%s Database NOP disconnect", __PRETTY_FUNCTION__);
}

void DBHandler::DBClose() {
    if (m_DBHandle != NULL) {
        Logger::info("%s closing Database", __PRETTY_FUNCTION__);
        sqlite3_close(m_DBHandle);
        m_DBHandle = NULL;
        // Ensure it closes properly (by sleeping a millisecond)?
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        m_databaseLock.unlock();
    } else {
        m_databaseLock.unlock();
        throw "DBHandler::DBClose() : connection is already null";
    }
}

int DBHandler::getTable(const std::string& sql,
                        std::vector<std::string>& results,
                        int& rows,
                        int& columns) {
    int resultCode = -1;
    sqlite3_stmt* statement = NULL;
    sqlite3* db;

    db = DBConnect();

    // prepare the statement sql code in byte form for query request
    if ((resultCode = sqlite3_prepare_v2(db, sql.c_str(), sql.size(), &statement, NULL)) !=
        SQLITE_OK) {  // if not OK, return error
        sqlite3_finalize(statement);
        return resultCode;
    }

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

int DBHandler::insertLog(std::string table, std::string values) {
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
}

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

std::vector<std::string> DBHandler::getTableIds(std::string table) {
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
}

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

    nextLongitude = atof(retrieveCell("current_Mission", results[1], "longitude").c_str());
    nextLatitude = atof(retrieveCell("current_Mission", results[1], "latitude").c_str());
    nextDeclination = retrieveCellAsInt("current_Mission", results[1], "declination");
    nextRadius = retrieveCellAsInt("current_Mission", results[1], "radius");
    nextStayTime = retrieveCellAsInt("current_Mission", results[1], "stay_time");
    isCheckpoint = retrieveCellAsInt("current_Mission", results[1], "is_checkpoint");

    if (foundPrev)  // Set values to next waypoint if harvested waypoint found
    {
        prevId = safe_stoi(results2[1]);

        prevLongitude = atof(retrieveCell("current_Mission", results2[1], "longitude").c_str());
        prevLatitude = atof(retrieveCell("current_Mission", results2[1], "latitude").c_str());
        prevDeclination = retrieveCellAsInt("current_Mission", results2[1], "declination");
        prevRadius = retrieveCellAsInt("current_Mission", results2[1], "radius");
    }

    return true;
}

std::string DBHandler::getConfigs() {
    Json js;

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

bool DBHandler::changeOneValue(std::string table,
                               std::string id,
                               std::string newValue,
                               std::string colName) {
    if (not DBTransaction("UPDATE " + table + " SET " + colName + " = " + newValue +
                          " WHERE id = " + id + ";")) {
        Logger::error("Error %s", __PRETTY_FUNCTION__);
        return false;
    }
    return true;
}
