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
	SECTION("Open a connection to an existing database")
	{
		DBHandler db("testdb.db");
		REQUIRE_NOTHROW(db.retrieveCell("mock", "1", "gps"));
	}

	SECTION("Open a connection to a non existing database")
	{
		DBHandler db("doesnotexist");
		REQUIRE_THROWS(db.retrieveCell("mock", "1", "gps"));
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

	SECTION("GetMinId function, should return 1")
	{
		DBHandler db("testdb.db");
		//Convert result from string to int
		REQUIRE(atoi(db.getIdFromTable("messages",false).c_str()) == 1);
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

	SECTION("retrieveCell function")
	{

		DBHandler db("testdb.db");

		REQUIRE(db.retrieveCell("waypoints","1","harvested").compare("0") == 0);

	}

	SECTION("retrieveCellAsInt function")
	{

		DBHandler db("testdb.db");

		REQUIRE(db.retrieveCellAsInt("waypoints","1","harvested") == 0);

	}

	SECTION("Delete row with id 1 from table waypoints")
	{
		DBHandler db("testdb.db");
		REQUIRE_NOTHROW(db.deleteRow("waypoints", "1"));
		//SHOULD ROLLBACK
	}

	SECTION("GetMinId function, should return an empty string")
	{
		DBHandler db("testdb.db");

		REQUIRE(db.getIdFromTable("waypoints",false).empty());
	}

	SECTION("Clear table function")
	{
		DBHandler db("testdb.db");

		REQUIRE(db.getIdFromTable("messages",false).compare("1") == 0);

		REQUIRE_NOTHROW(db.clearTable("messages"));

		REQUIRE(db.getIdFromTable("messages",false).empty());

	}

	SECTION("Insert function")
	{
		DBHandler db("testdb.db");

		REQUIRE_NOTHROW(db.insert("waypoints", "id, latitude, longitude, radius, harvested", "1, 2.2, 3.3, 500, 0"));

		REQUIRE(db.getIdFromTable("waypoints",false).compare("1") == 0);

		REQUIRE(db.retrieveCell("waypoints","1","radius").compare("500") == 0);

	}

	SECTION("changeOneValue function")
	{
		DBHandler db("testdb.db");

		REQUIRE(db.retrieveCell("waypoints","1","harvested").compare("0") == 0);

		REQUIRE_NOTHROW(db.changeOneValue("waypoints", "1","1","harvested"));

		REQUIRE(db.retrieveCell("waypoints","1","harvested").compare("1") == 0);

	}


	SECTION("Check value from scanning-waypoint")
	{
		DBHandler db("testdb.db");

		REQUIRE(db.retrieveCellAsInt("waypoint_index", "1", "id") == 0);
		REQUIRE_NOTHROW(db.insert("waypoint_index", "id, i, j", "1,1,3"));
		REQUIRE(db.retrieveCellAsInt("waypoint_index", "1", "id") == 1);
		REQUIRE(db.retrieveCellAsInt("waypoint_index", "2", "id") == 0);

	}


	SECTION("InsertScan function")
	{
		PositionModel m_pm(1.1, 2.2);
		DBHandler db("testdb.db");

		REQUIRE_NOTHROW(db.insert("waypoint_index", "id, i, j", "2,1,3"));

		REQUIRE(db.retrieveCell("waypoint_index", "2","i").compare("1") == 0);

		REQUIRE(db.retrieveCell("waypoint_index", "2","j").compare("3") == 0);

		REQUIRE_NOTHROW(db.insertScan("2",m_pm, 19.5,"1999-12-12"));

		REQUIRE(db.retrieveCell("scanning_measurements","1","air_temperature").compare("19.5") == 0);

		REQUIRE(db.retrieveCell("scanning_measurements","1","i").compare("1") == 0);

		REQUIRE(db.retrieveCell("scanning_measurements","1","j").compare("3") == 0);


		REQUIRE(db.retrieveCell("scanning_measurements","1","latitude").compare("1.1") == 0);

		REQUIRE(db.retrieveCell("scanning_measurements","1","longitude").compare("2.2") == 0);

		REQUIRE(db.retrieveCell("scanning_measurements","1","waypoint_id").compare("2") == 0);

		REQUIRE(db.retrieveCell("scanning_measurements","1","id").compare("1") == 0);


	}

	SECTION("Get/insert waypoint routing config") {
		DBHandler db("testdb.db");
		db.insert("waypoint_routing_config", "radius_ratio, sail_adjust_time, adjust_degree_limit", "1.5, 0.5, 2.5");

		REQUIRE(db.retrieveCell("waypoint_routing_config", "2", "radius_ratio").compare("1.5") == 0);
		REQUIRE(db.retrieveCell("waypoint_routing_config", "2", "sail_adjust_time").compare("0.5") == 0);
		REQUIRE(db.retrieveCell("waypoint_routing_config", "2", "adjust_degree_limit").compare("2.5") == 0);

	}

	SECTION("Get/insert course calulation config") {
		DBHandler db("testdb.db");
		db.insert("course_calculation_config", "tack_angle, tack_max_angle, tack_min_speed, sector_angle", "1.0,2.0,3.0,4.0");

		REQUIRE(db.retrieveCell("course_calculation_config", "2", "tack_angle").compare("1.0") == 0);
		REQUIRE(db.retrieveCell("course_calculation_config", "2", "tack_max_angle").compare("2.0") == 0);
		REQUIRE(db.retrieveCell("course_calculation_config", "2", "tack_min_speed").compare("3.0") == 0);
		REQUIRE(db.retrieveCell("course_calculation_config", "2", "sector_angle").compare("4.0") == 0);
	}

	SECTION("Get/insert sailing robot config") {
		DBHandler db("testdb.db");
		db.insert("sailing_robot_config", "flag_heading_compass, loop_time, scanning","21, 0.1, 1");

		REQUIRE(db.retrieveCell("sailing_robot_config", "2", "flag_heading_compass").compare("21") == 0);
		REQUIRE(db.retrieveCell("sailing_robot_config", "2", "loop_time").compare("0.1") == 0);
		REQUIRE(db.retrieveCell("sailing_robot_config", "2", "scanning").compare("1") == 0);
	}

	SECTION("Get/insert buffer config") {
		DBHandler db("testdb.db");
		db.insert("buffer_config", "compass, true_wind, windsensor","1,10,100");

		REQUIRE(db.retrieveCell("buffer_config", "2", "compass").compare("1") == 0);
		REQUIRE(db.retrieveCell("buffer_config", "2", "true_wind").compare("10") == 0);
		REQUIRE(db.retrieveCell("buffer_config", "2", "windsensor").compare("100") == 0);
	}

	SECTION("Get/insert wind vane config") {
		DBHandler db("testdb.db");
		db.insert("wind_vane_config", "use_self_steering, wind_sensor_self_steering, self_steering_interval","2,3,0.2");

		REQUIRE(db.retrieveCell("wind_vane_config", "2", "use_self_steering").compare("2") == 0);
		REQUIRE(db.retrieveCell("wind_vane_config", "2", "wind_sensor_self_steering").compare("3") == 0);
		REQUIRE(db.retrieveCell("wind_vane_config", "2", "self_steering_interval").compare("0.2") == 0);
	}

	SECTION("Get/insert maestro controller config") {
		DBHandler db("testdb.db");
		db.insert("maestro_controller_config", "port", "2222");

		REQUIRE(db.retrieveCell("maestro_controller_config", "2", "port").compare("2222") == 0);
	}

	SECTION("Get/insert rudder servo config") {
		DBHandler db("testdb.db");
		db.insert("rudder_servo_config", "channel, speed, acceleration", "2,3,4");

		REQUIRE(db.retrieveCell("rudder_servo_config","2","channel").compare("2") == 0);
		REQUIRE(db.retrieveCell("rudder_servo_config","2","speed").compare("3") == 0);
		REQUIRE(db.retrieveCell("rudder_servo_config","2","acceleration").compare("4") == 0);
	}

	SECTION("Get/insert sail servo config") {
		DBHandler db("testdb.db");
		db.insert("sail_servo_config", "channel, speed, acceleration", "10,11,12");

		REQUIRE(db.retrieveCell("sail_servo_config","2","channel").compare("10") == 0);
		REQUIRE(db.retrieveCell("sail_servo_config","2","speed").compare("11") == 0);
		REQUIRE(db.retrieveCell("sail_servo_config","2","acceleration").compare("12") == 0);
	}

	SECTION("Get/insert rudder command config") {
		DBHandler db("testdb.db");
		db.insert("rudder_command_config", "extreme_command, midship_command","2000,3000");

		REQUIRE(db.retrieveCell("rudder_command_config","2","extreme_command").compare("2000") == 0);
		REQUIRE(db.retrieveCell("rudder_command_config","2","midship_command").compare("3000") == 0);
	}

	SECTION("Get/insert sailing command config") {
		DBHandler db("testdb.db");
		db.insert("sail_command_config","close_reach_command, run_command" ,"0,0");

		REQUIRE(db.retrieveCell("sail_command_config","2","close_reach_command").compare("0") == 0);
		REQUIRE(db.retrieveCell("sail_command_config","2","run_command").compare("0") == 0);
	}

}
