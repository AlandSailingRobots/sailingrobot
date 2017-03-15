#include "catch.hpp"
#include <thread>
#include "waypointrouting/WaypointRouting.h"
#include "waypointrouting/Commands.h"
#include "waypointrouting/TackAngle.h"
#include "models/SystemStateModel.h"
#include "models/PositionModel.h"

#include <iostream>

TEST_CASE("testWaypointRoutingStandardCommandsCTS")
{
	double tackAngle = 45;
	double maxTackAngle = 60;
	double minTackSpeed = 0.5;
	double sectorAngle = 5;
	double twd = 180.0;
	double innerRadiusRatio = 1.0;
	int waypointRadius = 50;
	double heading = 340;
	double maxCommandAngle = 90.0;
	double rudderSpeedMin = 1.0;
	double rudder = 0, sail = 0;

	WindsensorModel windsensorModel(100, 0, 0);
	CompassModel compassModel(heading, 0, 0, AccelerationModel(0, 0, 0));
	GPSModel gpsModel("timestamp", PositionModel(0,0), 0, 0.1, heading, 1);
	SystemStateModel sysModel(gpsModel, windsensorModel, compassModel,AnalogArduinoModel(0,0,0,0), 0,0);

	WaypointModel waypoint(PositionModel(60.107597, 19.922965), waypointRadius, "1", 6);
	waypoint.time = 0;

	WaypointRouting wpr(waypoint, innerRadiusRatio, tackAngle, maxTackAngle,
		minTackSpeed, sectorAngle, maxCommandAngle,rudderSpeedMin);
	wpr.setUpdateInterval(0.5);
	wpr.setMinimumDegreeLimit(10);

	CourseCalculation cc;
	cc.setTackAngle(tackAngle);
	cc.setSectorAngle(sectorAngle);

	std::this_thread::sleep_for(std::chrono::milliseconds(550));
	SECTION("outside waypoint radius")
	{
		PositionModel boatPos(60.104320, 19.918823);
		wpr.getCommands(rudder, sail, boatPos, twd, heading, sysModel);
		cc.calculateCourseToSteer(boatPos, waypoint, twd);

		REQUIRE(rudder == Approx(0.58).epsilon(0.01));
		REQUIRE(sail == Approx(0.40).epsilon(0.01));
		REQUIRE(wpr.getCTS() == cc.getCTS());
		REQUIRE_FALSE(wpr.nextWaypoint(boatPos));
	}

	SECTION("inside waypoint radius")
	{
		PositionModel boatPos(60.107522, 19.923437);
		wpr.getCommands(rudder, sail, boatPos, twd, heading, sysModel);
		cc.calculateCourseToSteer(boatPos, waypoint, twd);

		REQUIRE(rudder == Approx(-0.58).epsilon(0.01));
		REQUIRE(sail == Approx(0.40).epsilon(0.01));
		REQUIRE(wpr.getCTS() == cc.getCTS());
		REQUIRE(wpr.nextWaypoint(boatPos));
	}
}


TEST_CASE("testWaypointRoutingTimedCommandsCTS")
{
	double tackAngle = 45;
	double maxTackAngle = 75;
	double minTackSpeed = 0.5;
	double sectorAngle = 5;
	double twd = 180.0;
	double innerRadiusRatio = 0.5;
	int waypointRadius = 50;
	double heading = 340;
	double maxCommandAngle = 90.0;
	double rudderSpeedMin = 1.0;
	double rudder = 0, sail = 0;

	WindsensorModel windsensorModel(100, 0, 0);
	CompassModel compassModel(heading, 0, 0, AccelerationModel(0, 0, 0) );
	GPSModel gpsModel("timestamp", PositionModel(0,0), 0, 0.1, heading, 1);
	SystemStateModel sysModel(gpsModel, windsensorModel, compassModel, AnalogArduinoModel(0,0,0,0), 0,0);

	WaypointModel waypoint(PositionModel(60.107597, 19.922965), waypointRadius, "1", 6);
	waypoint.time = 1;

	WaypointRouting wpr(waypoint, innerRadiusRatio, tackAngle, maxTackAngle,
		minTackSpeed, sectorAngle,maxCommandAngle,rudderSpeedMin);
	wpr.setUpdateInterval(0.5);
	wpr.setMinimumDegreeLimit(10);

	CourseCalculation cc;
	cc.setTackAngle(tackAngle);
	cc.setSectorAngle(sectorAngle);

	std::this_thread::sleep_for(std::chrono::milliseconds(550));
	SECTION("outside wp radius")
	{
		PositionModel boatPos(60.102564, 19.920614);
		wpr.getCommands(rudder, sail, boatPos, twd, heading, sysModel);
		cc.calculateCourseToSteer(boatPos, waypoint, twd);

		REQUIRE(rudder == Approx(0.36).epsilon(0.01));
		REQUIRE(sail == Approx(0.40).epsilon(0.01));
		REQUIRE(wpr.getCTS() == cc.getCTS());
		REQUIRE_FALSE(wpr.nextWaypoint(boatPos));
	}

	SECTION("inside wp radius, outside inner radius")
	{
		PositionModel boatPos(60.107608, 19.922246);
		wpr.getCommands(rudder, sail, boatPos, twd, heading, sysModel);
		cc.calculateCourseToSteer(boatPos, waypoint, twd);

		REQUIRE(rudder == 1);
		REQUIRE(sail == Approx(0.40).epsilon(0.01));
		REQUIRE(wpr.getCTS() == cc.getCTS());
		REQUIRE_FALSE(wpr.nextWaypoint(boatPos));
	}

	SECTION("inside inner radius")
	{
		PositionModel boatPos(60.107594, 19.922766);
		sysModel.windsensorModel.direction = 89; // changing rwd to trigger the steering
		wpr.getCommands(rudder, sail, boatPos, twd, heading, sysModel);

		REQUIRE(rudder == -1);
		REQUIRE(sail == 1);
		REQUIRE(wpr.getCTS() == twd);
		REQUIRE_FALSE(wpr.nextWaypoint(boatPos));
	}

}


TEST_CASE("testWaypointRoutingTimer")
{
	double tackAngle = 45;
	double maxTackAngle = 60;
	double minTackSpeed = 0.5;
	double sectorAngle = 5;
	double innerRadiusRatio = 0.5;
	double maxCommandAngle = 90.0;
	double rudderSpeedMin = 1.0;
	int waypointRadius = 50;

	WaypointModel waypoint(PositionModel(60.107597, 19.922965), waypointRadius, "1", 6);
	waypoint.time = 1;

	WaypointRouting wprInsideInner(waypoint, innerRadiusRatio, tackAngle, maxTackAngle,
		minTackSpeed, sectorAngle,maxCommandAngle,rudderSpeedMin);
	WaypointRouting wprInsideOuter(waypoint, innerRadiusRatio, tackAngle, maxTackAngle,
		minTackSpeed, sectorAngle,maxCommandAngle,rudderSpeedMin);
	WaypointRouting wprOutsideOuter(waypoint, innerRadiusRatio, tackAngle, maxTackAngle,
		minTackSpeed, sectorAngle,maxCommandAngle,rudderSpeedMin);
	WaypointRouting wprInsideInnerToInsideOuter(waypoint, innerRadiusRatio, tackAngle, maxTackAngle,
		minTackSpeed, sectorAngle,maxCommandAngle,rudderSpeedMin);
	WaypointRouting wprInsideOuterToInsideInner(waypoint, innerRadiusRatio, tackAngle, maxTackAngle,
		minTackSpeed, sectorAngle,maxCommandAngle,rudderSpeedMin);
	WaypointRouting wprOutsideOuterToInsideOuter(waypoint, innerRadiusRatio, tackAngle, maxTackAngle,
		minTackSpeed, sectorAngle,maxCommandAngle,rudderSpeedMin);
	WaypointRouting wprOutsideOuterToInsideInner(waypoint, innerRadiusRatio, tackAngle, maxTackAngle,
		minTackSpeed, sectorAngle,maxCommandAngle,rudderSpeedMin);

	SECTION("different types of combinations")
	{
		PositionModel boatPosInsideInner(60.107594, 19.922766);
		PositionModel boatPosInsideOuter(60.107608, 19.922246);
		PositionModel boatPosOutsideOuter(60.102564, 19.920614);

		REQUIRE_FALSE(wprInsideInner.nextWaypoint(boatPosInsideInner));
		REQUIRE_FALSE(wprInsideOuter.nextWaypoint(boatPosInsideOuter));
		REQUIRE_FALSE(wprOutsideOuter.nextWaypoint(boatPosOutsideOuter));
		REQUIRE_FALSE(wprInsideInnerToInsideOuter.nextWaypoint(boatPosInsideInner));
		REQUIRE_FALSE(wprInsideOuterToInsideInner.nextWaypoint(boatPosInsideOuter));
		REQUIRE_FALSE(wprOutsideOuterToInsideOuter.nextWaypoint(boatPosOutsideOuter));
		REQUIRE_FALSE(wprOutsideOuterToInsideInner.nextWaypoint(boatPosOutsideOuter));

		std::this_thread::sleep_for(std::chrono::milliseconds(900));

		REQUIRE_FALSE(wprInsideInner.nextWaypoint(boatPosInsideInner));
		REQUIRE_FALSE(wprInsideOuter.nextWaypoint(boatPosInsideOuter));
		REQUIRE_FALSE(wprOutsideOuter.nextWaypoint(boatPosOutsideOuter));
		REQUIRE_FALSE(wprInsideInnerToInsideOuter.nextWaypoint(boatPosInsideInner));
		REQUIRE_FALSE(wprInsideOuterToInsideInner.nextWaypoint(boatPosInsideOuter));
		REQUIRE_FALSE(wprOutsideOuterToInsideOuter.nextWaypoint(boatPosOutsideOuter));
		REQUIRE_FALSE(wprOutsideOuterToInsideInner.nextWaypoint(boatPosOutsideOuter));

		std::this_thread::sleep_for(std::chrono::milliseconds(200));

		REQUIRE(wprInsideInner.nextWaypoint(boatPosInsideInner));
		REQUIRE(wprInsideOuter.nextWaypoint(boatPosInsideOuter));
		REQUIRE_FALSE(wprOutsideOuter.nextWaypoint(boatPosOutsideOuter));
		REQUIRE(wprInsideInnerToInsideOuter.nextWaypoint(boatPosInsideOuter));
		REQUIRE(wprInsideOuterToInsideInner.nextWaypoint(boatPosInsideInner));
		REQUIRE_FALSE(wprOutsideOuterToInsideOuter.nextWaypoint(boatPosInsideOuter));
		REQUIRE_FALSE(wprOutsideOuterToInsideInner.nextWaypoint(boatPosInsideInner));
	}
}


TEST_CASE("testWaypointRoutingTackangleAtSlowSpeed")
{
	double tackAngle = 45;
	double maxTackAngle = 60;
	double minTackSpeed = 0.5;
	double sectorAngle = 5;
	double innerRadiusRatio = 1.0;
	int waypointRadius = 10;
	double maxCommandAngle = 90;
	double rudderSpeedMin = 1.0;
	double rudder = 0, sail = 0;

	WindsensorModel windsensorModel(100, 0, 0);
	CompassModel compassModel(0,0,0, AccelerationModel(0, 0, 0) );
	GPSModel gpsModel("timestamp", PositionModel(0,0), 0, 1, 0, 1);
	SystemStateModel sysModel(gpsModel, windsensorModel, compassModel, AnalogArduinoModel(0,0,0,0), 0,0);

	WaypointModel waypoint(PositionModel(60.107730, 19.922641), waypointRadius, "1", 6);
	waypoint.time = 0;

	WaypointRouting wpr(waypoint, innerRadiusRatio, tackAngle, maxTackAngle,
		minTackSpeed, sectorAngle,maxCommandAngle,rudderSpeedMin);

	SECTION("going starboard of adjusted tackangle")
	{
		double twd = 35;
		double heading = 350;
		sysModel.gpsModel.speed = 0.1;
		PositionModel boatPos(60.107188, 19.922641);
		wpr.getCommands(rudder, sail, boatPos, twd, heading, sysModel);

		REQUIRE(rudder < 0);
	}

	SECTION("going port of adjusted tackangle")
	{
		double twd = 340;
		double heading = 25;
		sysModel.gpsModel.speed = 0.1;
		PositionModel boatPos(60.107188, 19.922641);
		wpr.getCommands(rudder, sail, boatPos, twd, heading, sysModel);

		REQUIRE(rudder > 0);
	}

	SECTION("going at adjusted tackangle")
	{
		double twd = 340;
		double heading = 40;
		sysModel.gpsModel.speed = 0;
		PositionModel boatPos(60.107188, 19.922641);
		wpr.getCommands(rudder, sail, boatPos, twd, heading, sysModel);

		REQUIRE(rudder == Approx(0));
	}
}

TEST_CASE("testWaypointRoutingCommands")
{
	Commands commandHandler;

	SECTION("rudder")
	{
		double courseToSteer = 0;
		double maxCommandAngle = 90;
		for (double heading = 0; heading < 360; heading++)
		{
			if (heading == 0)
				REQUIRE(commandHandler.rudderCommand(courseToSteer, heading,maxCommandAngle ) == 0);
			else if (heading < 90)
				REQUIRE(commandHandler.rudderCommand(courseToSteer, heading,maxCommandAngle) < 0);
			else if (heading < 180) 
				REQUIRE(commandHandler.rudderCommand(courseToSteer, heading,maxCommandAngle) == -1);
			else if (heading == 180) 
				REQUIRE(abs(commandHandler.rudderCommand(courseToSteer, heading,maxCommandAngle )  ) == 1);
			else if (heading < 270)
				REQUIRE(commandHandler.rudderCommand(courseToSteer, heading,maxCommandAngle) == 1);
			else
				REQUIRE(commandHandler.rudderCommand(courseToSteer, heading,maxCommandAngle) > 0);
		}
		double heading = 365;
		REQUIRE(commandHandler.rudderCommand(courseToSteer, heading,maxCommandAngle) < 0);
		heading = -20;
		REQUIRE(commandHandler.rudderCommand(courseToSteer, heading,maxCommandAngle) > 0);
		heading = -45;
		REQUIRE(commandHandler.rudderCommand(courseToSteer, heading,maxCommandAngle) == Approx(0.500).epsilon(0.001));
		heading = 45;
		REQUIRE(commandHandler.rudderCommand(courseToSteer, heading,maxCommandAngle) == Approx(-0.500).epsilon(0.001));
	}

	SECTION("sail")
	{
		double relativeWindDirection = -10;
		REQUIRE(commandHandler.sailCommand(relativeWindDirection) == Approx(0.000).epsilon(0.001));
		relativeWindDirection = 0;
		REQUIRE(commandHandler.sailCommand(relativeWindDirection) == 0);
		relativeWindDirection = 90;
		REQUIRE(commandHandler.sailCommand(relativeWindDirection) == Approx(0.333).epsilon(0.001));
		relativeWindDirection = 180;
		REQUIRE(commandHandler.sailCommand(relativeWindDirection) == 1);
		relativeWindDirection = 190;
		REQUIRE(commandHandler.sailCommand(relativeWindDirection) == Approx(0.925).epsilon(0.001));
		relativeWindDirection = 270;
		REQUIRE(commandHandler.sailCommand(relativeWindDirection) == Approx(0.333).epsilon(0.001));
		relativeWindDirection = 360;
		REQUIRE(commandHandler.sailCommand(relativeWindDirection) == 0);
	}
}

TEST_CASE("testWaypointRoutingTackAngle")
{
	double tackAngle = 45;
	double maxTackAngle = 60;
	double minTackSpeed = 0.5;
	TackAngle tackAnglehandler(tackAngle, maxTackAngle, minTackSpeed);

	WindsensorModel windsensorModel(100, 0, 0);
	CompassModel compassModel(0,0,0, AccelerationModel(0, 0, 0) );
	GPSModel gpsModel("timestamp", PositionModel(0,0), 0, 0.9, 0, 1);
	SystemStateModel sysModel(gpsModel, windsensorModel, compassModel, AnalogArduinoModel(0,0,0,0), 0,0);

	SECTION("higher than mintack speed")
	{
		sysModel.compassModel.heading = 0;
		sysModel.gpsModel.speed = 1;

		for (sysModel.gpsModel.heading = 0; sysModel.gpsModel.heading < 360; sysModel.gpsModel.heading++)
		{
			if (sysModel.gpsModel.heading >= 90 && sysModel.gpsModel.heading <= 270)
				REQUIRE(tackAnglehandler.adjustedTackAngle(sysModel) == maxTackAngle);
			else
				REQUIRE(tackAnglehandler.adjustedTackAngle(sysModel) == tackAngle);
		}
	}

	SECTION("40 percent of mintack speed")
	{
		sysModel.compassModel.heading = 0;
		sysModel.gpsModel.speed = 0.2;

		for (sysModel.gpsModel.heading = 0; sysModel.gpsModel.heading < 360; sysModel.gpsModel.heading++)
		{
			if (sysModel.gpsModel.heading >= 90 && sysModel.gpsModel.heading <= 270)
				REQUIRE(tackAnglehandler.adjustedTackAngle(sysModel) == maxTackAngle);
			else
				REQUIRE(tackAnglehandler.adjustedTackAngle(sysModel) == 54);
		}
	}
}

TEST_CASE("testWaypointRoutingSteeringCertainTimepoints") 
{
	Timer timer;
	int waypointRadius = 10;
	double tackAngle = 45;
	double maxTackAngle = 60;
	double minTackSpeed = 0.5;
	double sectorAngle = 5;
	double innerRadiusRatio = 1.0;
	double maxCommandAngle = 90.0;
	double rudderSpeedMin = 1.0;

	WindsensorModel windsensorModel(100, 0, 0);
	CompassModel compassModel(0,0,0, AccelerationModel(0, 0, 0) );
	GPSModel gpsModel("timestamp", PositionModel(0,0), 0, 0.9, 0, 1);
	SystemStateModel sysModel(gpsModel, windsensorModel, compassModel, AnalogArduinoModel(0,0,0,0), 0,0);
	WaypointModel waypoint(PositionModel(60.107730, 19.922641), waypointRadius, "1", 6);

	WaypointRouting wpr(waypoint, innerRadiusRatio, tackAngle, maxTackAngle,
		minTackSpeed, sectorAngle,maxCommandAngle,rudderSpeedMin);
	wpr.setUpdateInterval(0.5);
	wpr.setMinimumDegreeLimit(10);

	SECTION("not changing steering within interval with wind change")
	{
		double rudder1 = 0, rudder2 = -1, sail1 = 0, sail2 = -1;
		double twd = 35, heading = 350;
		
		sysModel.gpsModel.speed = 0.1;
		PositionModel boatPos(60.107188, 19.922641);
		wpr.getCommands(rudder1, sail1, boatPos, twd, heading, sysModel);
		
		sysModel.windsensorModel.direction = 89;
		wpr.getCommands(rudder2, sail2, boatPos, twd, heading, sysModel);

		REQUIRE(sail1 == sail2);
	}
	SECTION("not changing steering after interval with no wind change")
	{
		double rudder1 = 0, rudder2 = -1, sail1 = 0, sail2 = -1;
		double twd = 35, heading = 350;
		
		sysModel.gpsModel.speed = 0.1;
		PositionModel boatPos(60.107188, 19.922641);
		wpr.getCommands(rudder1, sail1, boatPos, twd, heading, sysModel);
		
		std::this_thread::sleep_for(std::chrono::milliseconds(550));
		wpr.getCommands(rudder2, sail2, boatPos, twd, heading, sysModel);

		REQUIRE(sail1 == sail2);
	}
	SECTION("changing steering after interval with wind change")
	{
		double rudder2 = -1, sail2 = -1;
		double twd = 35, heading = 350;
		
		sysModel.gpsModel.speed = 0.1;
		PositionModel boatPos(60.107188, 19.922641);
	
		sysModel.windsensorModel.direction = 89;
		std::this_thread::sleep_for(std::chrono::milliseconds(550));
		wpr.getCommands(rudder2, sail2, boatPos, twd, heading, sysModel);

		REQUIRE(sail2 == Approx(0.32).epsilon(0.01));
	}
}

TEST_CASE("testWaypointRoutingSpeedLimitCalculation") {
	double tackAngle = 45;
	double maxTackAngle = 60;
	double minTackSpeed = 0.5;
	double sectorAngle = 5;
	double twd = 180.0;
	double innerRadiusRatio = 1.0;
	int waypointRadius = 50;
	double heading = 340;
	double compassHeading = 340;
	double gpsHeading = 340;
	double maxCommandAngle = 90.0; 
	double rudderSpeedMin = 1.0;
	double rudder = 0, sail = 0;

	WindsensorModel windsensorModel(100, 0, 0);
	CompassModel compassModel(compassHeading, 0, 0, AccelerationModel(0, 0, 0));
	GPSModel gpsModel("timestamp", PositionModel(0,0), 0, 0.1, gpsHeading, 1);
	SystemStateModel sysModel(gpsModel, windsensorModel, compassModel, AnalogArduinoModel(0,0,0,0), 0,0);

	WaypointModel waypoint(PositionModel(60.107597, 19.922965), waypointRadius, "1", 6);
	waypoint.time = 0;

	PositionModel boatPos(60.104320, 19.918823);
	SECTION("max command angle is 90.0") {
			WaypointRouting wpr1(waypoint, innerRadiusRatio, tackAngle, maxTackAngle,
		minTackSpeed, sectorAngle,maxCommandAngle,rudderSpeedMin);
		
		sysModel.gpsModel.speed = 2;

		wpr1.getCommands(rudder, sail, boatPos, twd, heading, sysModel);

		REQUIRE(rudder == Approx(0.29).epsilon(0.01));

		sysModel.gpsModel.speed = 3;

		wpr1.getCommands(rudder, sail, boatPos, twd, heading, sysModel);

		REQUIRE(rudder == Approx(0.19).epsilon(0.01));
	}
	SECTION("max command angle is 45.0") {
		maxCommandAngle = 45.0; 
		
		WaypointRouting wpr2(waypoint, innerRadiusRatio, tackAngle, maxTackAngle,
			minTackSpeed, sectorAngle,maxCommandAngle,rudderSpeedMin);

		sysModel.gpsModel.speed = 2;

		wpr2.getCommands(rudder, sail, boatPos, twd, heading, sysModel);

		REQUIRE(rudder == Approx(0.29*2).epsilon(0.01));

		sysModel.gpsModel.speed = 3;

		wpr2.getCommands(rudder, sail, boatPos, twd, heading, sysModel);

		REQUIRE(rudder == Approx(0.19*2).epsilon(0.01));
	}
	
}