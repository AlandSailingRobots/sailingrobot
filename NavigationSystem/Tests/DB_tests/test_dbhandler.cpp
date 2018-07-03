#include "catch.hpp"
#include "dbhandler/DBHandler.h"
#include <cstdio>
#include <stdio.h>
#include "models/PositionModel.h"
#include "models/WaypointModel.h"
#include "models/WindsensorModel.h"
#include "models/CompassModel.h"
#include "models/AccelerationModel.h"
#include "models/GPSModel.h"
#include "models/SystemStateModel.h"
#include "models/AnalogArduinoModel.h"

void printOutput (std::string text){
	std::cout<<"--------- result ---------\n\n";
	std::cout<<text<<"\n\n";
	std::cout<<"--------------------------\n\n";
}


TEST_CASE("DBhandler")
{
	// the sections are order dependent, and have dependency on each other
	// if you change the order, or erase a section, you will die


	// The first two tests looks funky because of how the connections are handled now
	// Just trust them....

	SECTION("Open a connection to a non existing database")
	{
		DBHandler db("doesnotexist");
		// REQUIRE_THROWS(db.selectFromAsText("mock", "gps", 1)); // No mock in DB
	}

	SECTION("Delete query on a table that doesnt exist")
	{
		DBHandler db("testdb.db");
		REQUIRE_THROWS(db.deleteRow("UnexistingTable", "1"));
	}

	SECTION("Insert new row to table message")
	{
		DBHandler db("testdb.db");
		REQUIRE_NOTHROW(db.insertMessageLog("TESTGPSTIME", "TESTTYPE", "TESTMSG"));
		//SHOULD ROLLBACK
	}

	SECTION("Test insertDataLog with 22 valid values")
	{

		SystemStateModel m_systemStateModel(
			GPSModel("",PositionModel(0,0),0,0,0,0),
			WindsensorModel(0,0,0),
			CompassModel(0,0,0,AccelerationModel(0,0,0)),
			AnalogArduinoModel(0,0,0,0),
			0,
			0
		);

		DBHandler db("testdb.db");

		db.insertDataLog(m_systemStateModel,
											0,
											0,
											0.0,
											0.0,
											0.0,
											0,
											0,
											0,
											0.0,
											0
											);

		// SHOULD ROLLBACK
	}

	SECTION("Getting waypointmodel from db")
	{
		WaypointModel m_waypointModel(PositionModel(0,0),0,"",0);

		DBHandler db("testdb.db");

		REQUIRE_NOTHROW(db.getWaypointFromTable(m_waypointModel));

	}

	SECTION("selectFromAsText function")
	{

		DBHandler db("testdb.db");

		REQUIRE(db.retrieveCell("current_Mission","1","harvested").compare("0") == 0);

	}

	// NOT OK!
	SECTION("selectFromAsText function")
	{

		DBHandler db("testdb.db");

		REQUIRE(db.retrieveCellAsInt("current_Mission","1","harvested") == 0);

	}

	SECTION("Delete row with id 1 from table current_Mission")
	{
		DBHandler db("testdb.db");
		REQUIRE_NOTHROW(db.deleteRow("current_Mission", "1"));
		//SHOULD ROLLBACK
	}

	SECTION("GetMinId function, should return an empty string")
	{
		DBHandler db("testdb.db");

		REQUIRE(db.getIdFromTable("current_Mission",false).empty());
	}

	SECTION("Insert function")
	{
		DBHandler db("testdb.db");

		REQUIRE_NOTHROW(db.insert("current_Mission", "id, isCheckpoint, latitude, longitude, radius, harvested", "1, 2.2, 3.3, 500, 0"));

		REQUIRE(db.getIdFromTable("current_Mission",false).compare("1") == 0);

		REQUIRE(db.retrieveCell("current_Mission","1","radius").compare("500") == 0);

	}

	SECTION("changeOneValue function")
	{
		DBHandler db("testdb.db");

		REQUIRE(db.retrieveCell("current_Mission","1","harvested").compare("0") == 0);

		REQUIRE_NOTHROW(db.changeOneValue("current_Mission", "1","1","harvested"));

		REQUIRE(db.retrieveCell("current_Mission","1","harvested").compare("1") == 0);

	}

	SECTION("InsertScan function")
	{
		PositionModel m_pm(1.1, 2.2);
		DBHandler db("testdb.db");

		REQUIRE_NOTHROW(db.insertScan("2",m_pm, 19.5,"1999-12-12"));

		REQUIRE(db.retrieveCell("scanning_measurements","1","air_temperature").compare("19.5") == 0);

		REQUIRE(db.retrieveCell("scanning_measurements","1","i").compare("1") == 0);

		REQUIRE(db.retrieveCell("scanning_measurements","1","j").compare("3") == 0);


		REQUIRE(db.retrieveCell("scanning_measurements","1","latitude").compare("1.1") == 0);

		REQUIRE(db.retrieveCell("scanning_measurements","1","longitude").compare("2.2") == 0);

		REQUIRE(db.retrieveCell("scanning_measurements","1","waypoint_id").compare("2") == 0);

		REQUIRE(db.retrieveCell("scanning_measurements","1","id").compare("1") == 0);


	}

	SECTION("Get/insert buffer config") {
		DBHandler db("testdb.db");
		db.insert("config_buffer", "compass","1");

		REQUIRE(db.retrieveCell("config_buffer", "2", "compass").compare("1") == 0);
	}

	SECTION("Get/insert wind vane config") {
		DBHandler db("testdb.db");
		db.insert("wind_vane_config", "use_self_steering, wind_sensor_self_steering, self_steering_interval","2,3,0.2");

		REQUIRE(db.retrieveCell("wind_vane_config", "2", "use_self_steering").compare("2") == 0);
		REQUIRE(db.retrieveCell("wind_vane_config", "2", "wind_sensor_self_steering").compare("3") == 0);
		REQUIRE(db.retrieveCell("wind_vane_config", "2", "self_steering_interval").compare("0.2") == 0);
	}

	SECTION("Get/insert course regulator config") {
		DBHandler db("testdb.db");
		db.insert("course_course_regulator","loop_time, maxRudderAngle, pGain, iGain" ,"0.5,25.0,0.2,0.3");

		REQUIRE(db.retrieveCell("course_course_regulator","2","loop_time").compare("0.5") == 0);
		REQUIRE(db.retrieveCell("course_course_regulator","2","maxRudderAngle").compare("25.0") == 0);
		REQUIRE(db.retrieveCell("course_course_regulator","2","pGain").compare("0.2") == 0);
		REQUIRE(db.retrieveCell("course_course_regulator","2","iGain").compare("0.3") == 0);
	}

}
