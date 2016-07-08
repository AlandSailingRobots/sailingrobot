#ifndef __DBHANDLER_H__
#define __DBHANDLER_H__ //__DATACOLLECT_H__

#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <sqlite3.h>
#include "logger/Logger.h"

#include "libs/json/src/json.hpp"
using Json = nlohmann::json;

class SystemStateModel;
class WaypointModel;
class PositionModel;

class DBHandler {

private:

	char* m_error;
	int m_latestDataLogId;
	std::string m_currentWaypointId = "";
	std::string m_filePath;

	//execute INSERT query and add new row into table
	void queryTable(std::string sqlINSERT);

	//retrieve data from given table/tables, return value is a C 2D char array
	//rows and columns also return values (through a reference) about rows and columns in the result set
	std::vector<std::string> retrieveFromTable(std::string sqlSELECT, int &rows, int &columns);

	//adds a table row into the json object as a array if array flag is true,
	//otherwise it adds the table row as a json object
	//id field is not obligatory, can be left empty
	void getDataAsJson(std::string select, std::string table, std::string key, std::string id, Json& json, bool useArray);

	//gets the id column from a given table
	std::vector<std::string> getTableIds(std::string table);

	//gets all datatable names related to "ending" string
	//used to fetch all tables ending with _datalogs or _config
	std::vector<std::string> getTableNames(std::string like);

	//gets information(for instance: name/datatype) about all columns
	std::vector<std::string> getColumnInfo(std::string info, std::string table);

	//help function used in insertDataLog
	int insertLog(std::string table, std::string values);

	// own implementation of deprecated sqlite3_get_table()
	int getTable(sqlite3* db, const std::string &sql, std::vector<std::string> &results, int &rows, int &columns);

	sqlite3* openDatabase();

	void closeDatabase(sqlite3* connection);


public:

	DBHandler(std::string filePath);
	~DBHandler(void);

	int getRows(std::string table);

	void insertDataLog(
		SystemStateModel systemState,
		int sailServoPosition,
		int rudderServoPosition,
		double courseCalculationDistanceToWaypoint,
		double courseCalculationBearingToWaypoint,
		double courseCalculationCourseToSteer,
		bool courseCalculationTack,
		bool courseCalculationGoingStarboard,
		int waypointId,
		double trueWindDirectionCalc,
		bool routeStarted);

	void insertMessageLog(std::string gps_time, std::string type, std::string msg);

	//updates table with json string (data)
	void updateTableJson(std::string table, std::string data);

	//updates table using values given
	void updateTable(std::string table, std::string column, std::string value, std::string id);

	void clearTable(std::string table);

	void updateConfigs(std::string configs);
	void updateWaypoints(std::string waypoints);

    //retrieve one value from a table as string
	std::string retrieveCell(std::string table, std::string id, std::string column);

    //retrieve one value from a table as integer
	int retrieveCellAsInt(std::string table, std::string id, std::string column);

	// returns all logs in database as json; supply onlyLatest to get only the ones with the highest id
	std::string getLogs(bool onlyLatest);

	void removeLogs(std::string data);

	void clearLogs();

	//get id from table returns either max or min id from table.
	//max = false -> min id
	//max = true -> max id
	std::string getIdFromTable(std::string table, bool max);

	void deleteRow(std::string table, std::string id);

	void getWaypointFromTable(WaypointModel &waypointModel);

	WaypointModel getPreviouslyHarvestedWaypoint();

	void insert(std::string table, std::string fields, std::string values);

	// inserts area scanning measurements into db
	void insertScan(std::string waypoint_id, PositionModel position, float temperature, std::string timestamp);

	void changeOneValue(std::string table, std::string id, std::string newValue, std::string colName);

	std::string getWaypoints();

	std::string getConfigs();

};

#endif
