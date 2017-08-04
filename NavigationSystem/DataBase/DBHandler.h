#ifndef __DBHANDLER_H__
#define __DBHANDLER_H__ //__DATACOLLECT_H__

#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <sqlite3.h>
#include "SystemServices/Logger.h"
#include "Messages/WindStateMsg.h"
#include <mutex>

#include "Libs/json/src/json.hpp"
using Json = nlohmann::json;

struct LogItem {
		int 	m_compassHeading;
		int 	m_compassPitch;
		int 	m_compassRoll;
		bool	m_gpsHasFix;
		bool	m_gpsOnline;
		double	m_gpsLat;
		double	m_gpsLon;
		double	m_gpsUnixTime;
		double	m_gpsSpeed;
		double	m_gpsCourse;
		int		m_gpsSatellite;
		float	m_windDir;
		float	m_windSpeed;
		float 	m_windTemp;
		int 	m_arduinoPressure;
		int 	m_arduinoRudder;
		int 	m_arduinoSheet;
		int 	m_arduinoBattery;
		double 	m_rudder;
		double 	m_sail;
		int 	m_sailServoPosition;
		int 	m_rudderServoPosition;
		double 	m_distanceToWaypoint;
		double 	m_bearingToWaypoint;
		double 	m_courseToSteer;
		bool 	m_tack;
		bool 	m_goingStarboard;
		int 	m_waypointId;
		double 	m_twd;
		bool 	m_routeStarted;
		std::string m_timestamp_str;
	};

class DBHandler {

private:

	char* m_error;
	int m_latestDataLogId;
	std::string m_currentWaypointId = "";
	std::string m_filePath;
	static std::mutex m_databaseLock;

	//execute INSERT query and add new row into table
	bool queryTable(std::string sqlINSERT);
	bool queryTable(std::string sqlINSERT, sqlite3* db);

	//retrieve data from given table/tables, return value is a C 2D char array
	//rows and columns also return values (through a reference) about rows and columns in the result set
	std::vector<std::string> retrieveFromTable(std::string sqlSELECT, int &rows, int &columns);
	std::vector<std::string> retrieveFromTable(std::string sqlSELECT, int &rows, int &columns,sqlite3* db);

	//adds a table row into the json object as a array if array flag is true,
	//otherwise it adds the table row as a json object
	//id field is not obligatory, can be left empty
	void getDataAsJson(std::string select, std::string table, std::string key, std::string id, Json& js, bool useArray);

	//gets the id column from a given table
	std::vector<std::string> getTableIds(std::string table);

	// gets all datatable names related to "ending" string
	//used to fetch all tables ending with _datalogs or _config
	std::vector<std::string> getTableNames(std::string like);

	//gets information(for instance: name/datatype) about all columns
	std::vector<std::string> getColumnInfo(std::string info, std::string table);

	//help function used in insertDataLog
	int insertLog(std::string table, std::string values, sqlite3* db);

	// own implementation of deprecated sqlite3_get_table()
	int getTable(sqlite3* db, const std::string &sql, std::vector<std::string> &results, int &rows, int &columns);

	sqlite3* openDatabase();

	void closeDatabase(sqlite3* connection);


public:

	DBHandler(std::string filePath);
	~DBHandler(void);

	bool initialise();

	int getRows(std::string table);

	void insertDataLogs(std::vector<LogItem>& logs);

	void insertMessageLog(std::string gps_time, std::string type, std::string msg);

	//updates table with json string (data)
	bool updateTableJson(std::string table, std::string data);
	bool updateTableJsonObject(std::string table, Json data);

	//updates table using values given
	bool updateTable(std::string table, std::string column, std::string value, std::string id);

	void clearTable(std::string table);

	void updateConfigs(std::string configs);
	bool updateWaypoints(std::string waypoints);

    //retrieve one value from a table as string
	std::string retrieveCell(std::string table, std::string id, std::string column);

    //retrieve one value from a table as integer
	int retrieveCellAsInt(std::string table, std::string id, std::string column);

	//retrieve one value from a table as double
	double retrieveCellAsDouble(std::string table, std::string id, std::string column);

	// returns all logs in database as json; supply onlyLatest to get only the ones with the highest id
	std::string getLogs(bool onlyLatest);

	void forceUnlock() { m_databaseLock.unlock(); }

	void clearLogs();

	//get id from table returns either max or min id from table.
	//max = false -> min id
	//max = true -> max id
	std::string getIdFromTable(std::string table, bool max);
	std::string getIdFromTable(std::string table, bool max,sqlite3* db);

	void deleteRow(std::string table, std::string id);

	bool getWaypointValues(int& nextId, double& nextLongitude, double& nextLatitude, int& nextDeclination, int& nextRadius, int& nextStayTime,
                        int& prevId, double& prevLongitude, double& prevLatitude, int& prevDeclination, int& prevRadius, bool& foundPrev);

	bool insert(std::string table, std::string fields, std::string values);

	// inserts area scanning measurements into db
	//TODO - remove here as well yeyeye
	//void insertScan(std::string waypoint_id, PositionModel position, float temperature, std::string timestamp);

	bool changeOneValue(std::string table, std::string id, std::string newValue, std::string colName);

	std::string getWaypoints();

	std::string getConfigs();

};

#endif
