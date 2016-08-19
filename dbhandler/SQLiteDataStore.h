/****************************************************************************************
 *
 * File:
 * 		SQLiteDataStore.h
 *
 * Purpose:
 *		
 *
 * Developer Notes:
 *
 ***************************************************************************************/


#pragma once

#include <sqlite3.h>

#include <string>
#include <vector>


enum class DataType {
	Integer,
	Float,
	String
};

struct StoreItem {
	std::string m_Key;
	DataType m_dataType;
	union {
		int m_int;
		float m_float;
		std::string* m_str;
	};

	StoreItem(std::string key, int i)
		:m_Key(key), m_dataType(DataType::Integer), m_int(i)
	{ }

	StoreItem(std::string key, float i)
		:m_Key(key), m_dataType(DataType::Float), m_float(i)
	{ }

	StoreItem(std::string key, std::string* i)
		:m_Key(key), m_dataType(DataType::String), m_str(i)
	{ }
};

class SQLiteDataStore {
public:
	SQLiteDataStore(std::string dbPath);
	virtual ~SQLiteDataStore();

	///----------------------------------------------------------------------------------
	/// Ensures that the SQLite DB is present and can be opened.
	///----------------------------------------------------------------------------------
	virtual bool initialise();

	///----------------------------------------------------------------------------------
	/// Reads a config float from the DB
	///----------------------------------------------------------------------------------
	virtual bool readConfigAsFloat(std::string section, std::string key, float& data);

	///----------------------------------------------------------------------------------
	/// Reads a config string from the DB
	///----------------------------------------------------------------------------------
	virtual bool readConfigAsString(std::string section, std::string key, std::string& data);

	///----------------------------------------------------------------------------------
	/// Reads a config integer from the DB
	///----------------------------------------------------------------------------------
	virtual bool readConfigAsInt(std::string section, std::string key, int& data);

	///----------------------------------------------------------------------------------
	/// Logs a set of key value pairs into a table, this is typical sensor data.
	///----------------------------------------------------------------------------------
	virtual bool logData(std::string section, std::vector<StoreItem>& items);

private:
	///----------------------------------------------------------------------------------
	/// Setups a new database
	///----------------------------------------------------------------------------------
	bool setupDB();

	std::string storeItemValueToString(StoreItem& item);

	///----------------------------------------------------------------------------------
	/// Returns a SQL statement that will read a a databse config value
	///----------------------------------------------------------------------------------
	std::string sqlStatementReadConfig(std::string table, std::string id);

	///----------------------------------------------------------------------------------
	/// Executes a SQL statement where the caller isn't expecting a response
	///----------------------------------------------------------------------------------
	bool executeSQL(std::string sqlStatement);

	///----------------------------------------------------------------------------------
	/// Executes a SQL statement where the caller is only expecting one response.
	///----------------------------------------------------------------------------------
	bool executeSQLWithResponse(std::string sqlStatement, std::string& result);

	///----------------------------------------------------------------------------------
	/// Executes a SQL statement where the caller is expecting more than one response
	///----------------------------------------------------------------------------------
	bool executeSQLWithResponse(std::string sqlStatement, std::vector<std::string>& result);

	std::string 	m_DBPath;
	sqlite3* 		m_DBHandle;
};
