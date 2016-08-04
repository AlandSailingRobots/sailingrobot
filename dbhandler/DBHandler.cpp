#include "DBHandler.h"
#include <iomanip>
#include <string>
#include <cstdlib>
#include <cstdio>
#include "models/SystemStateModel.h"
#include "models/WaypointModel.h"
#include "models/PositionModel.h"
#include "utility/Timer.h"


DBHandler::DBHandler(std::string filePath) :
	m_filePath(filePath)
{
	m_latestDataLogId = 1;
}


DBHandler::~DBHandler(void) {

}

bool DBHandler::initialise()
{
	sqlite3* connection = openDatabase();

	if(connection != 0)
	{
		closeDatabase(connection);
		return true;
	}
	else
	{
		return false;
	}
}


void DBHandler::getDataAsJson(std::string select, std::string table, std::string key, std::string id, Json& json, bool useArray) {
	int rows = 0, columns = 0;
	std::vector<std::string> values;
	std::vector<std::string> columnNames;
	std::vector<std::string> results;
	
	try {
		if(id == "")
			results = retrieveFromTable("SELECT " + select + " FROM " + table + ";", rows, columns);
		else
			results = retrieveFromTable("SELECT " + select + " FROM " + table + " WHERE ID = " + id + ";", rows, columns);
	} catch(const char * error) {
		Logger::error("%s, %s table: %s Error: %s", __PRETTY_FUNCTION__, select.c_str(), table.c_str(), error);
	}

	for(int i = 0; i < columns*(rows+1); ++i) {
		if(i < columns)
			columnNames.push_back(results[i]);
		else
			values.push_back(results[i]);
	}

	Json jsonEntry;
	for (int i = 0; i < rows; i++) {
		for(unsigned int j = 0; j < columnNames.size(); j++) {
			int index = j+(columns*i);
			jsonEntry[columnNames.at(j)] = values.at(index);
		}
			if(useArray) {
				if(!json[key].is_array()) {
					json[key] = Json::array({});
				}
				json[key].push_back(jsonEntry);
			} else {
				json[key] = jsonEntry;
			}

	}

	values.clear();
	columnNames.clear();
}

void DBHandler::insertDataLogs(std::vector<LogItem>& logs)
{
		std::stringstream arduinoValues;
		std::stringstream gpsValues;
		std::stringstream courseCalculationValues;
		std::stringstream compassModelValues;
		std::stringstream systemValues;
		std::stringstream windsensorValues;
		std::stringstream ss;
		int logNumber =0;
		int arduinoId=0;
		int windsensorId=0;
		int gpsId=0;
		int courceCalculationId=0;
		int compassModelId=0;
		std::string tableId;

		sqlite3* db = openDatabase();

		if(db == NULL)
		{
			Logger::error("%s Database is null!", __PRETTY_FUNCTION__);
			return;
		}

    if (logs.size()>0)
		{
		  Logger::info("Writing in the database last value: %s size logs %d",logs[0].m_timestamp_str.c_str(),logs.size());
    }

		tableId = getIdFromTable("arduino_datalogs",true,db);
		if(tableId.size() > 0)
		{
			arduinoId = (int)strtol(tableId.c_str(), NULL, 10);
		}
		tableId = getIdFromTable("windsensor_datalogs",true,db);
		if(tableId.size() > 0)
		{
			windsensorId = (int)strtol(tableId.c_str(), NULL, 10);
		}
		tableId = getIdFromTable("gps_datalogs",true,db);
		if(tableId.size() > 0)
		{
			gpsId = (int)strtol(tableId.c_str(), NULL, 10);
		}
		tableId = getIdFromTable("course_calculation_datalogs",true,db);
		if(tableId.size() > 0)
		{
			courceCalculationId = (int)strtol(tableId.c_str(), NULL, 10);
		}
		tableId = getIdFromTable("compass_datalogs",true,db);
		if(tableId.size() > 0)
		{
			compassModelId = (int)strtol(tableId.c_str(), NULL, 10);
		}


		for(auto log: logs)
		{
      logNumber++;
			arduinoValues.str("");
			gpsValues.str("");
			courseCalculationValues.str("");
			compassModelValues.str("");
			systemValues.str("");
			windsensorValues.str("");

			arduinoValues << std::setprecision(10)
				<< log.m_arduinoPressure << ", "
				<< log.m_arduinoRudder << ", "
				<< log.m_arduinoSheet << ", "
				<< log.m_arduinoBattery;

		  ss << "INSERT INTO " << "arduino_datalogs" << " VALUES(NULL, " << arduinoValues.str() << ");";

			gpsValues << std::setprecision(10) << "'"
				<< log.m_timestamp_str.c_str() << "', "
				<< log.m_gpsLat << ", "
				<< log.m_gpsLon << ", "
				<< log.m_gpsSpeed << ", "
				<< log.m_gpsHeading << ", "
				<< log.m_gpsSatellite << ", "
				<< log.m_routeStarted;

		  ss << "INSERT INTO " << "gps_datalogs" << " VALUES(NULL, " << gpsValues.str() << ");";

			courseCalculationValues << std::setprecision(10)
				<< log.m_distanceToWaypoint << ", "
				<< log.m_bearingToWaypoint << ", "
				<< log.m_courseToSteer << ", "
				<< log.m_tack << ", "
				<< log.m_goingStarboard;

		  ss << "INSERT INTO " << "course_calculation_datalogs" << " VALUES(NULL, " << courseCalculationValues.str() << ");";

			compassModelValues << std::setprecision(10)
				<< log.m_compassHeading << ", "
				<< log.m_compassPitch << ", "
				<< log.m_compassRoll;

		  ss << "INSERT INTO " << "compass_datalogs" << " VALUES(NULL, " << compassModelValues.str() << ");";

			windsensorValues << std::setprecision(10)
				<< log.m_windDir << ", "
				<< log.m_windSpeed << ", "
				<< log.m_windTemp;

		  ss << "INSERT INTO " << "windsensor_datalogs" << " VALUES(NULL, " << windsensorValues.str() << ");";

			systemValues << std::setprecision(10)
				<< gpsId+logNumber << ", "
				<< courceCalculationId+logNumber << ", "
				<< arduinoId+logNumber << ", "
				<< windsensorId+logNumber << ", "
				<< compassModelId+logNumber << ", "
				<< log.m_sail << ", "
				<< log.m_rudder << ", "
				<< log.m_sailServoPosition << ", "
				<< log.m_rudderServoPosition << ", "
				<< log.m_waypointId << ", "
				<< log.m_twd;


		  ss << "INSERT INTO " << "system_datalogs" << " VALUES(NULL, " << systemValues.str() << ");";
		}

		if(queryTable(ss.str(), db))
		{

			tableId = getIdFromTable("system_datalogs",true,db);
			if(tableId.size() > 0)
			{
				m_latestDataLogId = (int)strtol(tableId.c_str(), NULL, 10);
			}
			else
			{
				m_latestDataLogId = 0;
			}
		}
		else
		{
			m_latestDataLogId = 0;
			Logger::error("%s Error, failed to insert log Request: %s", __PRETTY_FUNCTION__,ss.str().c_str());
		}

	  closeDatabase(db);
}
//TODO -Oliver: make private
void DBHandler::insertMessageLog(std::string gps_time, std::string type, std::string msg) {
	std::string result;
	std::stringstream sstm;
	sstm << "INSERT INTO messages VALUES(NULL"
		<< ", '" << gps_time << "', '" << type << "', '" << msg << "', " << (m_latestDataLogId)
		<< ");";
	queryTable(sstm.str());
}

//TODO - make private
bool DBHandler::updateTableJson(std::string table, std::string data) {

	//m_logger.info(" updateTableJson:\n"+data);
	std::vector<std::string> columns = getColumnInfo("name", table);

	if(columns.size() <= 0 ){
		Logger::error("%s Error: no such table %s", __PRETTY_FUNCTION__, table.c_str());
		return false;
	}


	Json json = Json::parse(data);

	std::stringstream ss;

	//start at i = 1 to skip the id
	ss << "SET ";
	int fixedSize = json.size(); //Size would sometimes change, added this variable
	for (auto i = 1; i < fixedSize; i++) {
		if (fixedSize > 1){
			ss << columns.at(i) << " = " << json[columns.at(i)] << ","; //This crashes if the local database has fewer fields than the web database  (field out of range)
		}
	}

	std::string values = ss.str();
	values = values.substr(0, values.size()-1);

	std::string id = json["id"];

	if(not queryTable("UPDATE " + table + " " + values + " WHERE ID = " + id + ";"))
	{
		Logger::error("%s Error: ", __PRETTY_FUNCTION__);
		return false;
	}
	return true;
}

bool DBHandler::updateTable(std::string table, std::string column, std::string value, std::string id) {
	if(not queryTable("UPDATE " + table + " SET " + column + " = " + value + " WHERE ID = " + id + ";"))
	{
		Logger::error("%s Error updating table", __PRETTY_FUNCTION__);
		return false;
	}
	return true;
}


std::string DBHandler::retrieveCell(std::string table, std::string id, std::string column) {
	std::string query = "SELECT " + column + " FROM " + table +" WHERE id=" + id + ";";

	int rows, columns;
    std::vector<std::string> results;
    try {
    	results = retrieveFromTable(query, rows, columns);
    }
    catch(const char* error) {
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

	Json json = Json::parse(configs);

	std::vector<std::string> tables;

	for (auto i : Json::iterator_wrapper(json))  {
		tables.push_back(i.key()); //For each table key
	}

	//tables = sailing_config buffer_config etc

	for (auto table : tables) { //for each table in there
		if(json[table] != NULL){
			updateTableJson(table,json[table].dump()); //eg updatetablejson("sailing_config", configs['sailing_config'] as json)
		}
	}
}


bool DBHandler::updateWaypoints(std::string waypoints){

	Json json = Json::parse(waypoints);
	std::string DBPrinter = "";
	std::string tempValue = "";
	int valuesLimit = 6; //"Dirty" fix for limiting the amount of values requested from server waypoint entries (amount of fields n = valuesLimit + 1)
	int limitCounter;

	if(not queryTable("DELETE FROM waypoints;"))
	{
		Logger::error("%s, Error: failed to delete waypoints", __PRETTY_FUNCTION__);
	}


	for (auto i : Json::iterator_wrapper(json))  {
		//m_logger.info(i.value().dump());

		for (auto y : Json::iterator_wrapper(i.value())){

			limitCounter = valuesLimit;
			DBPrinter = "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (";

			for (auto z : Json::iterator_wrapper(y.value())){
				//Each individual value
				tempValue = z.value().dump();
				tempValue = tempValue.substr(1, tempValue.size() - 2);
				if (limitCounter > 0){
					limitCounter--;
					DBPrinter = DBPrinter + tempValue + ",";
				}

			}

			//if (DBPrinter.size () > 0)  DBPrinter.resize (DBPrinter.size () - 1);
			DBPrinter = DBPrinter + "0);";


			if(not queryTable(DBPrinter))
			{
				Logger::error("%s, Error: failed to add waypoints", __PRETTY_FUNCTION__);
				return false;
			}
		}
	}

	//Make sure waypoints before the current waypoint are harvested
	if (!m_currentWaypointId.empty()){
		std::string updateHarvested = "UPDATE waypoints SET harvested = 1 WHERE id < ";
		updateHarvested += m_currentWaypointId + ";";

		if(not queryTable(updateHarvested))
		{
			Logger::error("%s, Error: failed to harvest waypoints", __PRETTY_FUNCTION__);
			return false;
		}
	}
	return true;
}


int DBHandler::retrieveCellAsInt(std::string table, std::string id, std::string column) {

	std::string data = retrieveCell(table, id, column);
	if (data.size() > 0)
	{
		return strtol(data.c_str(), NULL, 10);
	}
	else
	{
		Logger::error("%s, Error: No data in cell ", __PRETTY_FUNCTION__);
		return 0;
	}

}


void DBHandler::clearTable(std::string table) {
	//If no table to delete, doesn't matter
	queryTable("DELETE FROM " + table + ";");
}

int DBHandler::getRows(std::string table) {
	int columns, rows;
	try {
		retrieveFromTable("SELECT * FROM " + table + ";", rows, columns);
	}
	catch(const char* error)
	{
		return 0;
	}
	return rows;
}

std::string DBHandler::getLogs(bool onlyLatest) {
	Json json;

	//fetch all datatables ending with "_datalogs"
	std::vector<std::string> datalogTables = getTableNames("%_datalogs");

	try {
		//insert all data in these tables as json array

		for (auto table : datalogTables) {

			if(onlyLatest){
				//Gets the log entry with the highest id
				getDataAsJson("*",table + " ORDER BY id DESC LIMIT 1",table,"",json,true);
			}else{
				getDataAsJson("*",table,table,"",json,true);
			}

		}

	} catch(const char * error) {
		Logger::error("%s, Error: %s", __PRETTY_FUNCTION__, error);
	}

	return json.dump();
}


void DBHandler::clearLogs() {
	std::vector<std::string> datalogTables = getTableNames("%_datalogs");

	for (auto table : datalogTables) {
		clearTable(table);
	}
}


void DBHandler::deleteRow(std::string table, std::string id) {
	queryTable("DELETE FROM " + table + " WHERE id = " + id + ";");
}

bool DBHandler::insert(std::string table, std::string fields, std::string values)
{
	if(not queryTable("INSERT INTO " + table + "(" + fields + ") VALUES(" + values + ");"))
	{
		Logger::error("%s, Failed to insert into table", __PRETTY_FUNCTION__);
		return false;
	}
	return true;
}
//TODO - Oliver/Jordan - REMOVE this function if not needed
//void DBHandler::insertScan(std::string waypoint_id, PositionModel position, float temperature, std::string timestamp)
//{
//	//std::string waypoint_id = getMinIdFromTable("waypoints");
//
//	std::string i = "null", j = "null";
//
//
//	i = retrieveCell("waypoint_index", waypoint_id, "i");
//	j = retrieveCell("waypoint_index", waypoint_id, "j");
//
//	Logger::error("%s, Error: %s", __PRETTY_FUNCTION__);
//
//
//	std::ostringstream fields;
//	fields << "waypoint_id,"
//		<< "time_UTC,"
//		<< "latitude,"
//		<< "longitude,"
//		<< "air_temperature,"
//		<< "i,"
//		<< "j";
//
//	std::ostringstream values;
//	values << waypoint_id << ",'"
//		<< timestamp << "',"
//		<< position.latitude << ","
//		<< position.longitude << ","
//		<< temperature << ","
//		<< i << ","
//		<< j;
//
//	insert("scanning_measurements", fields.str(), values.str());
//}

std::string DBHandler::getWaypoints() {
	int rows = 0;
	Json json;
	std::string wp = "waypoint_";

	rows = getRows("waypoints");
	if (rows > 0) {
		for (auto i = 1; i <= rows; ++i) {
			getDataAsJson("id,latitude,longitude,declination,radius,stay_time", "waypoints", wp + std::to_string(i), std::to_string(i),json, true);
		}
		return json.dump();
	}
	else
	{
		Logger::warning("No waypoints in database");
		return "";
	}
}

//get id from table returns either max or min id from table.
//max = false -> min id
//max = true -> max id
std::string DBHandler::getIdFromTable(std::string table, bool max) {
	int rows, columns;
    std::vector<std::string> results;
    try {
		if(max) {
			results = retrieveFromTable("SELECT MAX(id) FROM " + table + ";", rows, columns);
		} else {
			results = retrieveFromTable("SELECT MIN(id) FROM " + table + ";", rows, columns);
		}
}
    catch(const char* error) {
    	rows = 0;
    	columns = 0;
    }

    if (rows * columns < 1) {
    	return "";
    }
    if(results[1] == "\0") {
    	return "";
    } else {
    	return results[1];
    }
}

std::string DBHandler::getIdFromTable(std::string table, bool max,sqlite3* db) {
	int rows, columns;
    std::vector<std::string> results;
    try {
		if(max) {
			results = retrieveFromTable("SELECT MAX(id) FROM " + table + ";", rows, columns,db);
		} else {
			results = retrieveFromTable("SELECT MIN(id) FROM " + table + ";", rows, columns,db);
		}
}
    catch(const char* error) {
    	rows = 0;
    	columns = 0;
    }

    if (rows * columns < 1) {
    	return "";
    }
    if(results[1] == "\0") {
    	return "";
    } else {
    	return results[1];
    }
}


////////////////////////////////////////////////////////////////////
// private helpers
////////////////////////////////////////////////////////////////////

sqlite3* DBHandler::openDatabase() {
	sqlite3* connection;
	int resultcode = 0;

	// check if file exists
	FILE* db_file = fopen(m_filePath.c_str(), "r");
	if (!db_file) {
		Logger::error("%s %s not found", __PRETTY_FUNCTION__, m_filePath.c_str());
	}
	fclose(db_file);

	do {
		resultcode = sqlite3_open(m_filePath.c_str(), &connection);
	} while(resultcode == SQLITE_BUSY);

	if (resultcode) {
		Logger::error("%s Failed to open the database Error %s", __PRETTY_FUNCTION__, sqlite3_errmsg(connection));
		return 0;
	}

	// set a 10 millisecond timeout
	sqlite3_busy_timeout(connection, 10);
	return connection;
}


void DBHandler::closeDatabase(sqlite3* connection) {

	if(connection != NULL) {
		sqlite3_close(connection);
		connection = NULL;
	} else {
		throw "DBHandler::closeDatabase() : connection is already null";
	}
}

int DBHandler::getTable(sqlite3* db, const std::string &sql, std::vector<std::string> &results, int &rows, int &columns) {
	int resultcode = -1;
	sqlite3_stmt* statement = NULL;

	if((resultcode = sqlite3_prepare_v2(db, sql.c_str(), sql.size(), &statement, NULL)) != SQLITE_OK) {
		sqlite3_finalize(statement);
		return resultcode;
	}

	columns = sqlite3_column_count(statement);
	rows = 0;

	// read column names
	for(int i=0; i<columns; i++) {



		if(!sqlite3_column_name(statement, i)) {
			sqlite3_finalize(statement);
			return SQLITE_EMPTY;
		}

		results.emplace_back( const_cast<char*>(sqlite3_column_name(statement, i)) );
	}

	// read the rest of the table
	while( (resultcode = sqlite3_step(statement)) == SQLITE_ROW ) {

		for(int i=0; i<columns; i++) {
			if (results[i] != "dflt_value"){ //[es] Not a perfect solution. Needed for pragma sql statements as it is always null
				if(!sqlite3_column_text(statement, i)) {
					sqlite3_finalize(statement);
					rows = 0;
					columns = 0;
					return SQLITE_EMPTY;
				}

				results.emplace_back( reinterpret_cast<char*>(const_cast<unsigned char*>(sqlite3_column_text(statement, i))) );
			}else{
				results.emplace_back( "NULL" );
			}

		}
		rows++;
	}

	sqlite3_finalize(statement);

	if(resultcode != SQLITE_DONE) {
		return resultcode;
	}
	return SQLITE_OK;
}


int DBHandler::insertLog(std::string table, std::string values, sqlite3* db) {
	std::stringstream ss;
	ss << "INSERT INTO " << table << " VALUES(NULL, " << values << ");";

	if(queryTable(ss.str(), db))
	{
		Timer time_;
		time_.start();
		std::string tableId = getIdFromTable(table,true,db);
		time_.stop();
		Logger::info("Time passed writing %.5f",time_.timePassed());
		if(tableId.size() > 0)
		{
			return (int)strtol(tableId.c_str(), NULL, 10);
		}
	}
	else
	{
		Logger::error("%s Error, failed to insert log Request: %s", __PRETTY_FUNCTION__,ss.str().c_str());
	}

	return 0;
}

bool DBHandler::queryTable(std::string sqlINSERT) {
	sqlite3* db = openDatabase();
	m_error = NULL;

	if (db != NULL) {
		int resultcode = 0;

		do {
			if(m_error != NULL) {
				sqlite3_free(m_error);
				m_error = NULL;
			}

			resultcode = sqlite3_exec(db, sqlINSERT.c_str(), NULL, NULL, &m_error);
		} while(resultcode == SQLITE_BUSY);
		if (m_error != NULL) {
			Logger::error("%s Error: %s", __PRETTY_FUNCTION__, sqlite3_errmsg(db));

			sqlite3_free(m_error);
			return false;
		}
	}
	else {
		Logger::error("%s Error: no database found", __PRETTY_FUNCTION__);
		return false;
	}
	closeDatabase(db);
	return true;
}

bool DBHandler::queryTable(std::string sqlINSERT, sqlite3* db) {

	m_error = NULL;

	if (db != NULL) {
		int resultcode = 0;

		do {
			if(m_error != NULL) {
				sqlite3_free(m_error);
				m_error = NULL;
			}
			sqlite3_exec(db, "BEGIN TRANSACTION", NULL, NULL, &m_error);
			resultcode = sqlite3_exec(db, sqlINSERT.c_str(), NULL, NULL, &m_error);
			sqlite3_exec(db, "END TRANSACTION", NULL, NULL, &m_error);
		} while(resultcode == SQLITE_BUSY);
		if (m_error != NULL) {
			Logger::error("%s Error: %s", __PRETTY_FUNCTION__, sqlite3_errmsg(db));

			sqlite3_free(m_error);
			return false;
		}
	}
	else
	{
		Logger::error("%s Error: no database found", __PRETTY_FUNCTION__);
		return false;
	}

	return true;
}

std::vector<std::string> DBHandler::retrieveFromTable(std::string sqlSELECT, int &rows, int &columns) {
	sqlite3* db = openDatabase();
	std::vector<std::string> results;

	if (db != NULL) {
		int resultcode = 0;

		do {
			results = std::vector<std::string>();
			//resultcode = sqlite3_get_table(db, sqlSELECT.c_str(), &results, &rows, &columns, &m_error);
			resultcode = getTable(db, sqlSELECT, results, rows, columns);
		} while(resultcode == SQLITE_BUSY);

		if(resultcode == SQLITE_EMPTY) {
			std::vector<std::string> s;
			return s;
		}

		if (resultcode != SQLITE_OK) {
			Logger::error("%s SQL statement: %s Error: %s", __PRETTY_FUNCTION__, sqlSELECT.c_str(), sqlite3_errstr(resultcode));
			throw "retrieveFromTable";
		}
	}
	else {
		throw "DBHandler::retrieveFromTable(), no db connection";
	}

	closeDatabase(db);
	return results;
}

std::vector<std::string> DBHandler::retrieveFromTable(std::string sqlSELECT, int &rows, int &columns,sqlite3* db) {
	std::vector<std::string> results;

	if (db != NULL) {
		int resultcode = 0;

		do {
			results = std::vector<std::string>();
			//resultcode = sqlite3_get_table(db, sqlSELECT.c_str(), &results, &rows, &columns, &m_error);
			resultcode = getTable(db, sqlSELECT, results, rows, columns);
		} while(resultcode == SQLITE_BUSY);

		if(resultcode == SQLITE_EMPTY) {
			std::vector<std::string> s;
			return s;
		}

		if (resultcode != SQLITE_OK) {
			Logger::error("%s SQL statement: %s Error: %s", __PRETTY_FUNCTION__, sqlSELECT.c_str(), sqlite3_errstr(resultcode));
			throw "retrieveFromTable";
		}
	}
	else {
		throw "DBHandler::retrieveFromTable(), no db connection";
	}
	return results;
}

std::vector<std::string> DBHandler::getTableIds(std::string table) {
	int rows, columns;
    std::vector<std::string> results;
    try {
    results = retrieveFromTable("SELECT id FROM " + table + ";", rows, columns);
    }
    catch(const char* error)
    {}

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
    results = retrieveFromTable("SELECT name FROM sqlite_master WHERE type='table' AND name LIKE '"+ like +"';", rows, columns);
    }
    catch(const char* error) {

    }

    std::vector<std::string> tableNames;
    for (int i = 1; i <= rows; i++) {
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
	}
	catch(const char* error) {

	}

    std::vector<std::string> types;
    int infoIndex = 0;
    for (int i = 0; i < columns; i++) {
    	if (std::string(info).compare(results[i]) == 0) {
    		infoIndex = i;
    	}
    }

	for (int i = 1; i < rows+1; i++) {
		types.push_back(results[i * columns + infoIndex]);
	}
    return types;
}

bool DBHandler::getWaypointFromTable(WaypointModel &waypointModel, bool max){

	int rows, columns;
    std::vector<std::string> results;
    try {
    	if(max)
    	{
    		results = retrieveFromTable("SELECT MAX(id) FROM waypoints WHERE harvested = 1;", rows, columns);
    	}
    	else
    	{
    		results = retrieveFromTable("SELECT MIN(id) FROM waypoints WHERE harvested = 0;", rows, columns);
    	}
    }
    catch(const char* error)
    {
    	Logger::error("%s Error: %s", __PRETTY_FUNCTION__, error);
    	return false;
    }
    if (rows * columns < 1 || results[1] == "\0") {
    	return false;
    }


    waypointModel.id = results[1];

	m_currentWaypointId = waypointModel.id;
	waypointModel.positionModel.latitude = atof(retrieveCell("waypoints", waypointModel.id, "latitude").c_str());
	waypointModel.positionModel.longitude = atof(retrieveCell("waypoints", waypointModel.id, "longitude").c_str());
	waypointModel.radius = retrieveCellAsInt("waypoints", waypointModel.id, "radius");
	waypointModel.declination = retrieveCellAsInt("waypoints", waypointModel.id, "declination");

	results = retrieveFromTable("SELECT time FROM waypoint_stationary WHERE id = " +
		waypointModel.id + ";", rows, columns);

	if (rows * columns < 1 || results[1] == "\0") {
		waypointModel.time = 0;
	}
	else {
		waypointModel.time = retrieveCellAsInt("waypoint_stationary", waypointModel.id, "time");
	}

	return true;
}

bool DBHandler::getWaypointValues(int& nextId, double& nextLongitude, double& nextLatitude, int& nextDeclination, int& nextRadius, int& nextStayTime,
                        int& prevId, double& prevLongitude, double& prevLatitude, int& prevDeclination, int& prevRadius)
{
	int rows, columns, rows2, columns2;
    std::vector<std::string> results;
	std::vector<std::string> results2;
    try
    {
        results = retrieveFromTable("SELECT MIN(id) FROM waypoints WHERE harvested = 0;", rows, columns);
		results2 = retrieveFromTable("SELECT MAX(id) FROM waypoints WHERE harvested = 1;", rows2, columns2);
    }
    catch(const char* error)
    {
        Logger::error("%s Error: %s", __PRETTY_FUNCTION__, error);
        return false;
    }

    if (rows * columns < 1 || results[1] == "\0") {
        return false;
    }
	//Do not give values to previous waypoint if no value found in database
	bool foundPrevWaypoints = true;
    if (rows2 * columns2 < 1 || results2[1] == "\0") {
		Logger::info("No previously harvested waypoint found, values set as 0");
		foundPrevWaypoints = false;
    }

	//Set values to next waypoint
    nextId = stoi(results[1]);

    nextLongitude = atof(retrieveCell("waypoints", results[1], "longitude").c_str());
    nextLatitude = atof(retrieveCell("waypoints", results[1], "latitude").c_str());
    nextDeclination = retrieveCellAsInt("waypoints", results[1], "declination");
    nextRadius = retrieveCellAsInt("waypoints", results[1], "radius");
	nextStayTime = retrieveCellAsInt("waypoints", results[1], "stay_time");


	if(foundPrevWaypoints) //Set values to next waypoint if harvested waypoint found
	{
		prevId = stoi(results[1]);

		prevLongitude = atof(retrieveCell("waypoints", results2[1], "longitude").c_str());
		prevLatitude = atof(retrieveCell("waypoints", results2[1], "latitude").c_str());
		prevDeclination = retrieveCellAsInt("waypoints", results2[1], "declination");
		prevRadius = retrieveCellAsInt("waypoints", results2[1], "radius");
	}

    return true;
}

std::string DBHandler::getConfigs() {
	Json json;

	//Fetch all table names ending with "_config"
	std::vector<std::string> configTables = getTableNames("%_config");

	//Query config tables and select all from config tables with id "1"
	//This json structure does not use arrays
	for (auto table : configTables) {
		getDataAsJson("*",table,table,"1",json,false);
	}

	return json.dump();
}


bool DBHandler::changeOneValue(std::string table, std::string id,std::string newValue, std::string colName){

	if(not queryTable("UPDATE " + table + " SET "+ colName + " = "+ newValue +" WHERE id = " + id +";"))
	{
		Logger::error("Error %s", __PRETTY_FUNCTION__);
		return false;
	}
	return true;
}
