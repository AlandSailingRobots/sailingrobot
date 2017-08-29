#include "DBHandler.h"
#include <iomanip>
#include <string>
#include <cstdlib>
#include <cstdio>
#include "SystemServices/Timer.h"
#include <thread>


std::mutex DBHandler::m_databaseLock;


DBHandler::DBHandler(std::string filePath) :
	m_filePath(filePath)
{
	m_latestDataLogId = 0;
}

DBHandler::~DBHandler(void) {
	m_databaseLock.unlock();
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
		closeDatabase(connection);
		return false;
	}
}

void DBHandler::getDataAsJson(std::string select, std::string table, std::string key, std::string id, Json& js, bool useArray) {
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
		for(unsigned int j = 0; j < columnNames.size(); j++){
			int index = j+(columns*i);
			jsonEntry[columnNames.at(j)] = values.at(index);
		}

		if(useArray) {
			if(!js[key].is_array()){
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

void DBHandler::insertDataLogs(std::vector<LogItem>& logs)
{
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
		int actuatorFeedbackId=0;
		int compassModelId=0;
		int courceCalculationId=0;
		int currentMissionId=0;
		int currentSensorsId=0;
		int gpsId=0;
		int marineSensorsId=0;
		int vesselStateId=0;
		int windStateId=0;
		int windsensorId=0;
		int logNumber =0;
		std::string tableId;

		sqlite3* db = openDatabase();

		if(db == NULL)
		{
			Logger::error("%s Database is null!", __PRETTY_FUNCTION__);
			closeDatabase(db);
			return;
		}

    if (logs.size()>0)
		{
		  Logger::info("Writing in the database last value: %s size logs %d",logs[0].m_timestamp_str.c_str(),logs.size());
    	}

		tableId = getIdFromTable("dataLogs_actuator_feedback",true,db);
		if(tableId.size() > 0)
		{
			actuatorFeedbackId = (int)strtol(tableId.c_str(), NULL, 10);
		}
		tableId = getIdFromTable("dataLogs_compass",true,db);
		if(tableId.size() > 0)
		{
			compassModelId = (int)strtol(tableId.c_str(), NULL, 10);
		}
		tableId = getIdFromTable("dataLogs_course_calculation",true,db);
		if(tableId.size() > 0)
		{
			courceCalculationId = (int)strtol(tableId.c_str(), NULL, 10);
		}
		tableId = getIdFromTable("dataLogs_current_sensors",true,db);
		if(tableId.size() > 0)
		{
			currentSensorsId = (int)strtol(tableId.c_str(), NULL, 10);
		}
		tableId = getIdFromTable("dataLogs_gps",true,db);
		if(tableId.size() > 0)
		{
			gpsId = (int)strtol(tableId.c_str(), NULL, 10);
		}
		tableId = getIdFromTable("dataLogs_marine_sensors",true,db);
		if(tableId.size() > 0)
		{
			marineSensorsId = (int)strtol(tableId.c_str(), NULL, 10);
		}
		tableId = getIdFromTable("dataLogs_vessel_state",true,db);
		if(tableId.size() > 0)
		{
			vesselStateId = (int)strtol(tableId.c_str(), NULL, 10);
		}
		tableId = getIdFromTable("dataLogs_wind_state",true,db);
		if(tableId.size() > 0)
		{
			windStateId = (int)strtol(tableId.c_str(), NULL, 10);
		}
		tableId = getIdFromTable("dataLogs_windsensor",true,db);
		if(tableId.size() > 0)
		{
			windsensorId = (int)strtol(tableId.c_str(), NULL, 10);
		}
		// NOTE : Marc : To update the id of current_Mission in the DB
		tableId = getIdFromTable("current_Mission",true,db);
		if(tableId.size() > 0)
		{
			currentMissionId = (int)strtol(tableId.c_str(), NULL, 10);
		}


		for(auto log: logs)
		{

      logNumber++;
			actuatorFeedbackValues.str("");
			compassModelValues.str("");
			courseCalculationValues.str("");
			currentSensorsValues.str("");
			gpsValues.str("");
			marineSensorsValues.str("");
			vesselStateValues.str("");
			windStateValues.str("");
			windsensorValues.str("");
			systemValues.str("");

		  actuatorFeedbackValues << std::setprecision(10)
			  << log.m_rudderPosition << ", "
			  << log.m_wingsailPosition << ", "
			  << log.m_radioControllerOn << ", "
			  << log.m_windVaneAngle << ",'"
			  << log.m_timestamp_str.c_str();

		  ss << "INSERT INTO " << "dataLogs_actuator_feedback" << " VALUES(NULL, " << actuatorFeedbackValues.str() << "'); \n";

		  compassModelValues << std::setprecision(10)
			  << log.m_compassHeading << ", "
			  << log.m_compassPitch << ", "
			  << log.m_compassRoll<< ",'"
			  << log.m_timestamp_str.c_str();

		  ss << "INSERT INTO " << "dataLogs_compass" << " VALUES(NULL, " << compassModelValues.str() << "'); \n";

		  courseCalculationValues << std::setprecision(10)
			  << log.m_distanceToWaypoint << ", "
			  << log.m_bearingToWaypoint << ", "
			  << log.m_courseToSteer << ", "
			  << log.m_tack << ", "
			  << log.m_goingStarboard<< ",'"
			  << log.m_timestamp_str.c_str();

		  ss << "INSERT INTO " << "dataLogs_course_calculation" << " VALUES(NULL, " << courseCalculationValues.str() << "'); \n";

		  currentSensorsValues << std::setprecision(10)
			  << log.m_currentActuatorUnit << ", "
			  << log.m_currentNavigationUnit << ", "
			  << log.m_currentWindVaneAngle << ", "
			  << log.m_currentWindVaneClutch << ", "
			  << log.m_currentSailboatDrive << ",'"
			  << log.m_timestamp_str.c_str();

	      ss << "INSERT INTO " << "dataLogs_current_sensors" << " VALUES(NULL, " << currentSensorsValues.str() << "'); \n";

		  gpsValues << std::setprecision(10)
			  << log.m_gpsHasFix << ", "
			  << log.m_gpsOnline <<",'"
			  << log.m_timestamp_str.c_str() << "', "
			  << log.m_gpsLat << ", "
			  << log.m_gpsLon << ", "
			  << log.m_gpsSpeed << ", "
			  << log.m_gpsCourse << ", "
			  << log.m_gpsSatellite << ", "
			  << log.m_routeStarted << ",'"
			  << log.m_timestamp_str.c_str();

		  ss << "INSERT INTO " << "dataLogs_gps" << " VALUES(NULL, " << gpsValues.str() << "'); \n";

		  marineSensorsValues << std::setprecision(10)
  			  << log.m_temperature << ", "
  			  << log.m_conductivity << ", "
			  << log.m_ph << ",'"
			  << log.m_timestamp_str.c_str();

	      ss << "INSERT INTO " << "dataLogs_marine_sensors" << " VALUES(NULL, " << marineSensorsValues.str() << "'); \n";

		  vesselStateValues << std::setprecision(10)
			  << log.m_vesselHeading << ", "
			  << log.m_vesselLat << ", "
		      << log.m_vesselLon << ", "
			  << log.m_vesselSpeed << ", "
			  << log.m_vesselCourse << ",'"
			  << log.m_timestamp_str.c_str();

		  ss << "INSERT INTO " << "dataLogs_vessel_state" << " VALUES(NULL, " << vesselStateValues.str() << "'); \n";

		  windStateValues << std::setprecision(10)
			  << log.m_trueWindSpeed << ", "
			  << log.m_trueWindDir << ", "
			  << log.m_apparentWindSpeed << ", "
			  << log.m_apparentWindDir << ",'"
			  << log.m_timestamp_str.c_str();

		  ss << "INSERT INTO " << "dataLogs_wind_state" << " VALUES(NULL, " << windStateValues.str() << "'); \n";


		  windsensorValues << std::setprecision(10)
			  << log.m_windDir << ", "
			  << log.m_windSpeed << ", "
			  << log.m_windTemp << ",'"
			  << log.m_timestamp_str.c_str();

		  ss << "INSERT INTO " << "dataLogs_windsensor" << " VALUES(NULL, " << windsensorValues.str() << "'); \n";

		  systemValues << std::setprecision(10)
			  << actuatorFeedbackId+logNumber << ", "
		      << compassModelId+logNumber << ", "
			  << courceCalculationId+logNumber << ", "
			  << currentSensorsId+logNumber << ", "
			  << gpsId+logNumber << ", "
			  << marineSensorsId+logNumber << ", "
			  << vesselStateId+logNumber << ", "
		  	  << windStateId+logNumber << ", "
			  << windsensorId+logNumber << ", "
			  << currentMissionId;

		  ss << "INSERT INTO " << "dataLogs_system" << " VALUES(NULL, " << systemValues.str() << "); \n";
		}

		if(queryTable(ss.str(), db))
		{
			tableId = getIdFromTable("dataLogs_system",true,db);
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
	//std::string result;
	//std::stringstream sstm;
	//sstm << "INSERT INTO messages VALUES(NULL"
		//<< ", '" << gps_time << "', '" << type << "', '" << msg << "', " << (m_latestDataLogId) // Not use in DataBase
		//<< ");";
	//queryTable(sstm.str());
}


bool DBHandler::updateTableJson(std::string table, std::string data) {

	std::vector<std::string> columns = getColumnInfo("name", table);

	if(columns.size() <= 0 ){
		Logger::error("%s Error: no such table %s", __PRETTY_FUNCTION__, table.c_str());
		return false;
	}


	Json js = Json::parse(data);

	std::stringstream ss;

	//start at i = 1 to skip the id
	ss << "SET ";
	int fixedSize = js.size(); //Size would sometimes change, added this variable
	for (auto i = 1; i < fixedSize; i++) {
		if (fixedSize > 1){
			ss << columns.at(i) << " = " << js[columns.at(i)] << ","; //This crashes if the local database has fewer fields than the web database  (field out of range)
		}
	}

	std::string values = ss.str();
	values = values.substr(0, values.size()-1);

	std::string id = js["id"];

	if(not queryTable("UPDATE " + table + " " + values + " WHERE ID = " + id + ";"))
	{
		Logger::error("%s Error: ", __PRETTY_FUNCTION__);
		return false;
	}
	return true;
}

bool DBHandler::updateTableJsonObject(std::string table, Json data) {


	//m_logger.info(" updateTableJson:\n"+data);
	std::vector<std::string> columns = getColumnInfo("name", table);

	if(columns.size() <= 0 ){
		Logger::error("%s Error: no such table %s", __PRETTY_FUNCTION__, table.c_str());
		return false;
	}


	//Json json = data;

	std::stringstream ss;

	//start at i = 1 to skip the id
	ss << "SET ";
	int fixedSize = data.size(); //Size would sometimes change, added this variable
	for (auto i = 1; i < fixedSize; i++) {
		if (fixedSize > 1){
			ss << columns.at(i) << " = " << data[columns.at(i)] << ","; //This crashes if the local database has fewer fields than the web database  (field out of range)
		}
	}

	std::string values = ss.str();
	values = values.substr(0, values.size()-1);

	std::string id = data["id"];

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
	Json js = Json::parse(configs);

	std::vector<std::string> tables;

	for (auto i : Json::iterator_wrapper(js))  {
		tables.push_back(i.key()); //For each table key
	}

	//tables = sailing_config config_buffer etc

	for (auto table : tables) { //for each table in there
		if(js[table] != NULL){
			updateTableJson(table,js[table].dump()); //eg updatetablejson("sailing_config", configs['sailing_config'] as json)
		}
	}
}

bool DBHandler::updateWaypoints(std::string waypoints){
	Json js = Json::parse(waypoints);
	std::string DBPrinter = "";
	std::string tempValue = "";
	int valuesLimit = 11; //"Dirty" fix for limiting the amount of values requested from server waypoint entries (amount of fields n = valuesLimit + 1)
	int limitCounter;

	if(not queryTable("DELETE FROM current_Mission;"))
	{
		Logger::error("%s, Error: failed to delete waypoints", __PRETTY_FUNCTION__);
	}


	for (auto i : Json::iterator_wrapper(js))  {
		//m_logger.info(i.value().dump());

		for (auto y : Json::iterator_wrapper(i.value())){

			limitCounter = valuesLimit;
			DBPrinter = "INSERT INTO current_Mission (declination,harvested,id,id_mission,is_checkpoint,latitude,longitude,name,radius,rankInMission,stay_time) VALUES (";

			for (auto z : Json::iterator_wrapper(y.value())){
				//Each individual value
				tempValue = z.value().dump();
				tempValue = tempValue.substr(1, tempValue.size() - 2);
				if (tempValue == "")
				{
					tempValue = "NULL";
				}
				if (limitCounter > 0){
					limitCounter--;
					DBPrinter = DBPrinter + tempValue + ",";
				}

			}

			//if (DBPrinter.size () > 0)  DBPrinter.resize (DBPrinter.size () - 1);
			//DBPrinter = DBPrinter + "0);";
			DBPrinter = DBPrinter.substr(0, DBPrinter.size()-1) + ");";
			std::cout << DBPrinter << "\n";
			if(not queryTable(DBPrinter))
			{
				Logger::error("%s, Error: failed to add waypoints", __PRETTY_FUNCTION__);
				return false;
			}
		}
	}

	//Make sure waypoints before the current waypoint are harvested
	if (!m_currentWaypointId.empty()){
		std::string updateHarvested = "UPDATE current_Mission SET harvested = 1 WHERE id < ";
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

double DBHandler::retrieveCellAsDouble(std::string table, std::string id, std::string column) {

	std::string data = retrieveCell(table, id, column);
	if (data.size() > 0)
	{
		return strtod(data.c_str(), NULL);
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
	Json js;

	//fetch all datatables starting with "dataLogs_"
	std::vector<std::string> datalogTables = getTableNames("dataLogs_%");

	try {
		//insert all data in these tables as json array

		for (auto table : datalogTables) {
			if(onlyLatest){
				//Gets the log entry with the highest id
				getDataAsJson("*",table + " ORDER BY id DESC LIMIT 1",table,"",js,true);
			}else{
				getDataAsJson("*",table,table,"",js,true);
			}

		}

	} catch(const char * error) {
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

std::string DBHandler::getWaypoints() { // NOTE : Marc : change this otherwise it doesn't work
	int rows = 0;
	Json js;
	std::string wp = "waypoint_";

	rows = getRows("current_Mission");
	// std::cout << "rows current mission " << rows << std::endl;
	if (rows > 0) {
		for (auto i = 1; i <= rows; ++i) {
			//getDataAsJson("id,is_checkpoint,latitude,longitude,declination,radius,stay_time", "current_Mission", wp + std::to_string(i), std::to_string(i),json, true);
			getDataAsJson("id,is_checkpoint,latitude,longitude,declination,radius,stay_time", "current_Mission", wp + std::to_string(i), std::to_string(i),js, true);
		}
		return js.dump();
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

std::string DBHandler::getIdFromTable(std::string table, bool max, sqlite3* db) {
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

	m_databaseLock.lock();
	sqlite3* connection;
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
		resultcode = sqlite3_open(m_filePath.c_str(), &connection);
	} while(resultcode == SQLITE_BUSY);

	if (resultcode) {
		Logger::error("%s Failed to open the database Error %s", __PRETTY_FUNCTION__, sqlite3_errmsg(connection));
		m_databaseLock.unlock();
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
		// ENsure it closes properly
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		m_databaseLock.unlock();
	} else {
		m_databaseLock.unlock();
		throw "DBHandler::closeDatabase() : connection is already null";
	}
}

int DBHandler::getTable(sqlite3* db, const std::string &sql, std::vector<std::string> &results, int &rows, int &columns) {
	int resultcode = -1;
	sqlite3_stmt* statement = NULL;

	// prepare the statement sql code in byte form for query request
	if((resultcode = sqlite3_prepare_v2(db, sql.c_str(), sql.size(), &statement, NULL)) != SQLITE_OK) { // if not OK, return error
		sqlite3_finalize(statement);
		return resultcode;
	}

	// get the number of columns int the table called in the statement
	columns = sqlite3_column_count(statement);
	rows = 0;

	// read column names
	for(int i=0; i<columns; i++) {

		// if column name is NULL, return error
		if(!sqlite3_column_name(statement, i)) {
			sqlite3_finalize(statement);
			return SQLITE_EMPTY;
		}

		// add to the result
		results.emplace_back( const_cast<char*>(sqlite3_column_name(statement, i)) );
	}

	// read the rest of the table
	while( (resultcode = sqlite3_step(statement)) == SQLITE_ROW )
	{

		for(int i=0; i<columns; i++) {
			if (results[i] != "dflt_value") //[es] Not a perfect solution. Needed for pragma sql statements as it is always null
			{
				// Get the value in the column
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

	sqlite3_finalize(statement); //destruct the statement

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
			closeDatabase(db);
			return false;
		}
	}
	else {
		Logger::error("%s Error: no database found", __PRETTY_FUNCTION__);
		closeDatabase(db);
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
			closeDatabase(db);
			return s;
		}

		if (resultcode != SQLITE_OK) {
			Logger::error("%s SQL statement: %s Error: %s", __PRETTY_FUNCTION__, sqlSELECT.c_str(), sqlite3_errstr(resultcode));
			closeDatabase(db);
			throw "retrieveFromTable";
		}
	}
	else {
		closeDatabase(db);
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

bool DBHandler::getWaypointValues(int& nextId, double& nextLongitude, double& nextLatitude, int& nextDeclination, int& nextRadius, int& nextStayTime,
                        int& prevId, double& prevLongitude, double& prevLatitude, int& prevDeclination, int& prevRadius, bool& foundPrev)
{
	int rows, columns, rows2, columns2;
    std::vector<std::string> results;
	std::vector<std::string> results2;
    try
    {
        results = retrieveFromTable("SELECT MIN(id) FROM current_Mission WHERE harvested = 0;", rows, columns);
		results2 = retrieveFromTable("SELECT MAX(id) FROM current_Mission WHERE harvested = 1;", rows2, columns2);
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
	foundPrev = true;
    if (rows2 * columns2 < 1 || results2[1] == "\0") {
		Logger::info("No previously harvested waypoint found, values set as 0");
		foundPrev = false;
    }


	//Set values to next waypoint
    nextId = stoi(results[1]);

    nextLongitude = atof(retrieveCell("current_Mission", results[1], "longitude").c_str());
    nextLatitude = atof(retrieveCell("current_Mission", results[1], "latitude").c_str());
    nextDeclination = retrieveCellAsInt("current_Mission", results[1], "declination");
    nextRadius = retrieveCellAsInt("current_Mission", results[1], "radius");
	nextStayTime = retrieveCellAsInt("current_Mission", results[1], "stay_time");


	if(foundPrev) //Set values to next waypoint if harvested waypoint found
	{
		prevId = stoi(results[1]);

		prevLongitude = atof(retrieveCell("current_Mission", results2[1], "longitude").c_str());
		prevLatitude = atof(retrieveCell("current_Mission", results2[1], "latitude").c_str());
		prevDeclination = retrieveCellAsInt("current_Mission", results2[1], "declination");
		prevRadius = retrieveCellAsInt("current_Mission", results2[1], "radius");
	}

    return true;
}

std::string DBHandler::getConfigs() {
	Json js;

	//Fetch all table names ending with "_config"
	std::vector<std::string> configTables = getTableNames("config_%"); // NOTE : Marc : Modify this point

	//Query config tables and select all from config tables with id "1"
	//This json structure does not use arrays
	for (auto table : configTables) {
		getDataAsJson("*",table,table,"1",js,false);
	}

	return js.dump();
}


bool DBHandler::changeOneValue(std::string table, std::string id,std::string newValue, std::string colName){

	if(not queryTable("UPDATE " + table + " SET "+ colName + " = "+ newValue +" WHERE id = " + id +";"))
	{
		Logger::error("Error %s", __PRETTY_FUNCTION__);
		return false;
	}
	return true;
}
