/****************************************************************************************
 *
 * File:
 * 		SQLiteDataStore.cpp
 *
 * Purpose:
 *		
 *
 * Developer Notes:
 *
 ***************************************************************************************/


#include "SQLiteDataStore.h"
#include "SystemServices/Logger.h"
#include <stdio.h>      /* printf, NULL */
#include <stdlib.h>     /* strtof */
#include <iostream>
#include <fstream>
#include <sstream>

#include <fstream>
#include <streambuf>
#include <cerrno>

#define SQL_TEST_VALUE	14101993

#define SQLITE3_OK 		0
#define SQLITE3_ERROR	1
#define SQLITE3_ROW		100


#define TABLE_APP_INFO	"app_info"


//---------------------------------------------------------------------------------------
std::vector<std::string> getFileContents(const char *filename)
{
	std::vector<std::string> lines;
	std::ifstream file(filename);
	std::string str;
	while (std::getline(file, str))
	{
	  lines.push_back(str);
	}
	return lines;
}


//---------------------------------------------------------------------------------------
SQLiteDataStore::SQLiteDataStore(std::string dbPath)
	:m_DBPath(dbPath), m_DBHandle(0)
{

}

//---------------------------------------------------------------------------------------
SQLiteDataStore::~SQLiteDataStore()
{
	if(m_DBHandle != NULL)
	{
		sqlite3_close(m_DBHandle);
	}
}

//---------------------------------------------------------------------------------------
bool SQLiteDataStore::initialise()
{
	if(m_DBHandle != NULL)
	{
		Logger::warning("Failed to initialise the database as it has already been initialised");
		return true;
	}

	// Check if the file exists
	FILE* db_file = fopen(m_DBPath.c_str(), "r");
	int result = sqlite3_open(m_DBPath.c_str(), &m_DBHandle);
	if (db_file == NULL)
	{
		Logger::info("Database file not found, creating a new one!");

		return setupDB();
	}
	else
	{
		fclose(db_file);
	}

	if(result == SQLITE3_ERROR)
	{
		Logger::error("%s:%d Failed to open the database: %s", __FILE__, __LINE__, sqlite3_errmsg(m_DBHandle));
		return false;
	}

	return true;
}

//---------------------------------------------------------------------------------------
bool SQLiteDataStore::readConfigAsFloat(std::string section, std::string key, float& data)
{
	std::string result;
	if(readConfigAsString(section, key, result))
	{
		data = strtof(result.c_str(), NULL);
		return true;
	}
	return true;
}

//---------------------------------------------------------------------------------------
bool SQLiteDataStore::readConfigAsString(std::string section, std::string key, std::string& data)
{
	std::string statement = sqlStatementReadConfig(section, key);

	if(not executeSQLWithResponse(statement, data))
	{
		return false;
	}

	return true;
}

//---------------------------------------------------------------------------------------
bool SQLiteDataStore::readConfigAsInt(std::string section, std::string key, int& data)
{
	std::string result;
	if(readConfigAsString(section, key, result))
	{
		data = std::stoi(result);
		return true;
	}
	return false;
}

//---------------------------------------------------------------------------------------
bool SQLiteDataStore::logData(std::string section, std::vector<StoreItem>& items)
{
	std::string sqlStatement = "INSERT INTO " + section + "(";

	for(unsigned int i = 0; i < items.size(); i++)
	{
		sqlStatement += items.at(i).m_Key;

		if(i + 1 < items.size())
		{
			sqlStatement += ",";
		}
	}

	sqlStatement += ") VALUES (";

	for(unsigned int i = 0; i < items.size(); i++)
	{
		sqlStatement += storeItemValueToString(items.at(i));

		if(i + 1 < items.size())
		{
			sqlStatement += ",";
		}
	}

	sqlStatement += ");";

	return executeSQL(sqlStatement);
}

//---------------------------------------------------------------------------------------
bool SQLiteDataStore::setupDB()
{
	std::vector<std::string> createTableLines = getFileContents("./CreateTables.sql");
	if(createTableLines.size() > 0)
	{
		for(unsigned int i = 0; i < createTableLines.size(); i++)
		{
			if(not executeSQL(createTableLines.at(i)))
			{
				return false;
			}
		}

		std::vector<StoreItem> test;

		test.push_back(StoreItem("thisEntryIsForTests", SQL_TEST_VALUE));
		return logData("app_info", test);
	}
	else
	{
		Logger::error("%s:%d Failed to open the create tables file: %s", __FILE__, __LINE__, "./CreateTables.sql");
		return false;
	}
}

//---------------------------------------------------------------------------------------
std::string SQLiteDataStore::storeItemValueToString(StoreItem& item)
{
	std::string value;

	switch(item.m_dataType)
	{
	case DataType::Integer:
		value = std::to_string(item.m_int);
		break;
	case DataType::Float:
		value = std::to_string(item.m_float);
		break;
	case DataType::String:
		value.assign(*item.m_str);
		break;
	}

	return value;
}

//---------------------------------------------------------------------------------------
std::string SQLiteDataStore::sqlStatementReadConfig(std::string table, std::string key)
{
	return "SELECT " + key + " FROM " + table + " WHERE id=1;";
}

//---------------------------------------------------------------------------------------
bool SQLiteDataStore::executeSQL(std::string sqlStatement)
{
	sqlite3_stmt* statement = NULL;
	int result = 0;

	if(sqlite3_prepare_v2(m_DBHandle, sqlStatement.c_str(), sqlStatement.size(), &statement, NULL) == SQLITE3_ERROR)
	{
		Logger::error("%s:%d Failed to generate a SQL statement", __FILE__, __LINE__);
		goto failed;
	}

	result = sqlite3_step(statement);

	if(result == SQLITE3_ERROR)
	{
		Logger::error("%s:%d step failed", __FILE__, __LINE__);
		goto failed;
	}
	sqlite3_finalize(statement);
	return true;

failed:
	sqlite3_finalize(statement);
	return false;
}

//---------------------------------------------------------------------------------------
bool SQLiteDataStore::executeSQLWithResponse(std::string sqlStatement, std::string& result)
{
	sqlite3_stmt* statement = NULL;
	int columns = 0;

	if(sqlite3_prepare_v2(m_DBHandle, sqlStatement.c_str(), sqlStatement.size(), &statement, NULL) == SQLITE3_ERROR)
	{
		Logger::error("%s:%d Failed to generate a SQL statement", __FILE__, __LINE__);
		goto failed;
	}

	columns = sqlite3_column_count(statement);

	if(columns > 0)
	{
		if(sqlite3_step(statement) != SQLITE3_ROW)
		{
			Logger::error("%s:%d No data", __FILE__, __LINE__);
			goto failed;
		}

		char* str = (char*)sqlite3_column_text(statement, 0);

		if(str == 0)
		{
			Logger::error("%s:%d No data", __FILE__, __LINE__);
			goto failed;
		}

		result.assign(str);
		sqlite3_finalize(statement);
		return true;
	}

failed:
	sqlite3_finalize(statement);
	return false;
}

//---------------------------------------------------------------------------------------
bool SQLiteDataStore::executeSQLWithResponse(std::string sqlStatement, std::vector<std::string>& result)
{
	//TODO - Jordan: SQLiteDataStore::executeSQLWithResponse(std::string sqlStatement, std::vector<std::string>& result)
	return false;
}
