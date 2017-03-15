#include "catch.hpp"
#include "Math/Utility.h"
#include "SystemServices/Timer.h"
#include <stdint.h> // uint8_t
#include <thread>

TEST_CASE("testUtility")
{
	SECTION("Combine bytes")
	{
		uint8_t MSB = 1; // = 256
		uint8_t LSB = 4;
		int combined = Utility::combineBytes(MSB, LSB);
		REQUIRE(combined == 260);
	}


	SECTION("Median even")
	{
		float f[] = {1.0, 2.0, 20.0, 0.0, 5.0, 5.0};
		std::vector<float> v (f, f + sizeof(f) / sizeof(float) );
		float middle = Utility::getMedianValue(v);
		REQUIRE(middle == 3.5);
	}
	SECTION("Median odd")
	{
		float f[] = {1.0, 2.0, 20.0, 5.0, 5.0};
		std::vector<float> v (f, f + sizeof(f) / sizeof(float) );
		float middle = Utility::getMedianValue(v);
		REQUIRE(middle == 5);
	}
	SECTION("Median even")
	{
		float f[] = {5.0};
		std::vector<float> v (f, f + sizeof(f) / sizeof(float) );
		float middle = Utility::getMedianValue(v);
		REQUIRE(middle == 5);
	}
	SECTION("Median empty vector")
	{
		std::vector<float> v;
		float middle = Utility::getMedianValue(v);
		REQUIRE(middle == 0);
	}
	SECTION("Median even")
	{
		float f[] = {80, 280};
		std::vector<float> v (f, f + sizeof(f) / sizeof(float) );
		float middle = Utility::getMedianValue(v);
		REQUIRE(middle == 0);
	}
	SECTION("Median even")
	{
		float f[] = {355, 357, 359, 1, 3 };
		std::vector<float> v (f, f + sizeof(f) / sizeof(float) );
		float middle = Utility::getMedianValue(v);
		REQUIRE(middle == 359);
	}


	SECTION("Mean")
	{
		float f[] = {1,2,3};
		std::vector<float> values (f, f + sizeof(f) / sizeof(float) );
		REQUIRE(Utility::mean(values) == 2);

		float f2[] = {1,2};
		std::vector<float> values2 (f2, f2 + sizeof(f2) / sizeof(float) );
		REQUIRE(Utility::mean(values2) == 1.5);
	}
	SECTION("Mean empty vector")
	{
		std::vector<float> v;
		REQUIRE(Utility::mean(v) == 0);
	}


	SECTION("Mean of angles")
	{
		float f[] = {0,180};
		std::vector<float> values (f, f + sizeof(f) / sizeof(float) );
		REQUIRE(Utility::meanOfAngles(values) == 90);
	}
	SECTION("Mean of angles around 0 degrees")
	{
		float f[] = {330,30};
		std::vector<float> values (f, f + sizeof(f) / sizeof(float) );
		REQUIRE(Utility::meanOfAngles(values) == 0);
	}
	SECTION("Mean of angles empty vector")
	{
		std::vector<float> v;
		REQUIRE(Utility::meanOfAngles(v) == 0);
	}
	SECTION("Mean of angles greater than 360")
	{
		float f[] = {370};
		std::vector<float> values (f, f + sizeof(f) / sizeof(float) );
		REQUIRE(Utility::meanOfAngles(values) == 10);
	}
	SECTION("Mean of angles less than 0")
	{
		float f[] = {-60};
		std::vector<float> values (f, f + sizeof(f) / sizeof(float) );
		REQUIRE(Utility::meanOfAngles(values) == 300);
	}


	SECTION("Polar to cartesian coordinates")
	{
		float x, y;
		Utility::polarToCartesian(0, x, y);
		REQUIRE(x == 1);
		REQUIRE(y == 0);
	}

	SECTION("if an angle is between 1st and 2nd angles given in arguments")
	{
		REQUIRE(Utility::isAngleInSector(10, 0, 30));
		REQUIRE(Utility::isAngleInSector(10, 0, 184));
		REQUIRE(Utility::isAngleInSector(10, 340, 40));
		REQUIRE(Utility::isAngleInSector(350, 340, 40));
		REQUIRE(Utility::isAngleInSector(10, -40, 40));
		REQUIRE(Utility::isAngleInSector(10, 0, 185));
		REQUIRE_FALSE(Utility::isAngleInSector(10, 40, 340));
		REQUIRE_FALSE(Utility::isAngleInSector(-10, 0, 30));
	}

	SECTION("difference between angles")
	{
		REQUIRE(Utility::angleDifference(359, 0) == 1);
		REQUIRE(Utility::angleDifference(0, 359) == 1);
		REQUIRE(Utility::angleDifference(0, 190) == 170);
		REQUIRE(Utility::angleDifference(500, 1070) == 150);
		REQUIRE(Utility::angleDifference(1070, 500) == 150);
		REQUIRE(Utility::angleDifference(-10, 10) == 20);
		REQUIRE(Utility::angleDifference(-1070, -500) == 150);
	}

	SECTION("limit anglerange")
	{
		REQUIRE(Utility::limitAngleRange(1) == 1);
		REQUIRE(Utility::limitAngleRange(359) == 359);
		REQUIRE(Utility::limitAngleRange(360) == 0);
		REQUIRE(Utility::limitAngleRange(0) == 0);
		REQUIRE(Utility::limitAngleRange(-1) == 359);
		REQUIRE(Utility::limitAngleRange(620) == 260);
		REQUIRE(Utility::limitAngleRange(2050) == 250);
		REQUIRE(Utility::limitAngleRange(-2050) == 110);
	}

	SECTION("Degree/radian conversion")
	{
		REQUIRE(Utility::degreeToRadian(0.0) == 0.0);
		REQUIRE(Utility::degreeToRadian(180.0) == Approx(M_PI));
		REQUIRE(Utility::degreeToRadian(90.0) == Approx(M_PI/2));
		REQUIRE(Utility::degreeToRadian(360.0) == Approx(M_PI*2));
	}

	SECTION("Degree/radian conversion")
	{
		REQUIRE(Utility::radianToDegree(0.0) == 0.0);
		REQUIRE(Utility::radianToDegree(1) == Approx(57.2957795));
		REQUIRE(Utility::radianToDegree(M_PI) == Approx(180.0));
	}
	SECTION("Adding declination to heading")
	{
		REQUIRE(Utility::addDeclinationToHeading(45, 6) == 51);
		REQUIRE(Utility::addDeclinationToHeading(0, -6) == 354);
		REQUIRE(Utility::addDeclinationToHeading(355, 6) == 1);
	}
}

TEST_CASE("TrueWindDirection"){

	SECTION("TrueWindDirection"){

		SystemStateModel model = SystemStateModel(
					GPSModel("",PositionModel(0,0),0,8,0,0),
					WindsensorModel(20,5/1.94384,0),
					CompassModel(0,0,0,AccelerationModel(0,0,0) ),
					AnalogArduinoModel(0,0,0,0),
					0,
					0
				);
		double heading = 181;

		double twd = Utility::calculateTrueWindDirection(model,heading);

		REQUIRE(twd == Approx(334).epsilon(0.01));

		model = SystemStateModel(
					GPSModel("",PositionModel(0,0),0,8,0,0),
					WindsensorModel(340,5/1.94384,0),
					CompassModel(0,0,0,AccelerationModel(0,0,0) ),
					AnalogArduinoModel(0,0,0,0),
					0,
					0
				);
		heading = 181;

		twd = Utility::calculateTrueWindDirection(model,heading);

		REQUIRE(twd == Approx(28.3).epsilon(0.01));

		model = SystemStateModel(
					GPSModel("",PositionModel(0,0),0,8,0,0),
					WindsensorModel(20,5/1.94384,0),
					CompassModel(0,0,0,AccelerationModel(0,0,0) ),
					AnalogArduinoModel(0,0,0,0),
					0,
					0
				);
		heading = 179;

		twd = Utility::calculateTrueWindDirection(model,heading);

		REQUIRE(twd == Approx(332).epsilon(0.01));

		model = SystemStateModel(
					GPSModel("",PositionModel(0,0),0,8,0,0),
					WindsensorModel(340,5/1.94384,0),
					CompassModel(0,0,0,AccelerationModel(0,0,0) ),
					AnalogArduinoModel(0,0,0,0),
					0,
					0
				);
		heading = 179;

		twd = Utility::calculateTrueWindDirection(model,heading);

		REQUIRE(twd == Approx(26.3).epsilon(0.01));

		model = SystemStateModel(
					GPSModel("",PositionModel(0,0),0,8,0,0),
					WindsensorModel(20,5/1.94384,0),
					CompassModel(0,0,0,AccelerationModel(0,0,0) ),
					AnalogArduinoModel(0,0,0,0),
					0,
					0
				);
		heading = 180;

		twd = Utility::calculateTrueWindDirection(model,heading);

		REQUIRE(twd == Approx(333).epsilon(0.01));

		model = SystemStateModel(
					GPSModel("",PositionModel(0,0),0,8,0,0),
					WindsensorModel(340,5/1.94384,0),
					CompassModel(0,0,0,AccelerationModel(0,0,0) ),
					AnalogArduinoModel(0,0,0,0),
					0,
					0
				);
		heading = 180;

		twd = Utility::calculateTrueWindDirection(model,heading);

		REQUIRE(twd == Approx(27.3).epsilon(0.01));

		model = SystemStateModel(
					GPSModel("",PositionModel(0,0),0,8,0,0),
					WindsensorModel(90,10/1.94384,0),
					CompassModel(0,0,0,AccelerationModel(0,0,0) ),
					AnalogArduinoModel(0,0,0,0),
					0,
					0
				);
		heading = 270;

		twd = Utility::calculateTrueWindDirection(model,heading);

		REQUIRE(twd == Approx(39).epsilon(0.01));

		model = SystemStateModel(
					GPSModel("",PositionModel(0,0),0,8,0,0),
					WindsensorModel(90,10/1.94384,0),
					CompassModel(0,0,0,AccelerationModel(0,0,0) ),
					AnalogArduinoModel(0,0,0,0),
					0,
					0
				);
		heading = 310;

		twd = Utility::calculateTrueWindDirection(model,heading);

		REQUIRE(twd == Approx(79).epsilon(0.01));

		model = SystemStateModel(
					GPSModel("",PositionModel(0,0),0,8,0,0),
					WindsensorModel(90,10/1.94384,0),
					CompassModel(0,0,0,AccelerationModel(0,0,0) ),
					AnalogArduinoModel(0,0,0,0),
					0,
					0
				);
		heading = 240;

		twd = Utility::calculateTrueWindDirection(model,heading);

		REQUIRE(twd == Approx(8.6).epsilon(0.01));

		model = SystemStateModel(
					GPSModel("",PositionModel(0,0),0,8,0,0),
					WindsensorModel(300,10/1.94384,0),
					CompassModel(0,0,0,AccelerationModel(0,0,0) ),
					AnalogArduinoModel(0,0,0,0),
					0,
					0
				);
		heading = 240;

		twd = Utility::calculateTrueWindDirection(model,heading);

		REQUIRE(twd == Approx(131).epsilon(0.01));

		model = SystemStateModel(
					GPSModel("",PositionModel(0,0),0,8,0,0),
					WindsensorModel(340,10/1.94384,0),
					CompassModel(0,0,0,AccelerationModel(0,0,0) ),
					AnalogArduinoModel(0,0,0,0),
					0,
					0
				);
		heading = 240;

		twd = Utility::calculateTrueWindDirection(model,heading);

		REQUIRE(twd == Approx(172).epsilon(0.01));

		model = SystemStateModel(
					GPSModel("",PositionModel(0,0),0,8,0,0),
					WindsensorModel(1,10/1.94384,0),
					CompassModel(0,0,0,AccelerationModel(0,0,0) ),
					AnalogArduinoModel(0,0,0,0),
					0,
					0
				);
		heading = 240;

		twd = Utility::calculateTrueWindDirection(model,heading);

		REQUIRE(twd == Approx(245).epsilon(0.01));

		model = SystemStateModel(
					GPSModel("",PositionModel(0,0),0,8,0,0),
					WindsensorModel(359,10/1.94384,0),
					CompassModel(0,0,0,AccelerationModel(0,0,0) ),
					AnalogArduinoModel(0,0,0,0),
					0,
					0
				);
		heading = 240;

		twd = Utility::calculateTrueWindDirection(model,heading);

		REQUIRE(twd == Approx(235).epsilon(0.01));


		model = SystemStateModel(
					GPSModel("",PositionModel(0,0),0,8,0,0),
					WindsensorModel(359,10/1.94384,0),
					CompassModel(0,0,0,AccelerationModel(0,0,0) ),
					AnalogArduinoModel(0,0,0,0),
					0,
					0
				);
		heading = 180;

		twd = Utility::calculateTrueWindDirection(model,heading);

		REQUIRE(twd == Approx(175).epsilon(0.01));


		model = SystemStateModel(
					GPSModel("",PositionModel(0,0),0,8,0,0),
					WindsensorModel(180,10/1.94384,0),
					CompassModel(0,0,0,AccelerationModel(0,0,0) ),
					AnalogArduinoModel(0,0,0,0),
					0,
					0
				);
		heading = 180;

		twd = Utility::calculateTrueWindDirection(model,heading);

		REQUIRE(twd == Approx(0).epsilon(0.01));

		//speed 0
		model = SystemStateModel(
					GPSModel("",PositionModel(0,0),0,0,0,0),
					WindsensorModel(0,10/1.94384,0),
					CompassModel(0,0,0,AccelerationModel(0,0,0) ),
					AnalogArduinoModel(0,0,0,0),
					0,
					0
				);
		heading = 180;

		twd = Utility::calculateTrueWindDirection(model,heading);

		REQUIRE(twd == Approx(180).epsilon(0.01));

		model = SystemStateModel(
					GPSModel("",PositionModel(0,0),0,0,0,0),
					WindsensorModel(360,10/1.94384,0),
					CompassModel(0,0,0,AccelerationModel(0,0,0) ),
					AnalogArduinoModel(0,0,0,0),
					0,
					0
				);
		heading = 180;

		twd = Utility::calculateTrueWindDirection(model,heading);

		REQUIRE(twd == Approx(180).epsilon(0.01));

		// wind 0
		model = SystemStateModel(
					GPSModel("",PositionModel(0,0),0,0,0,0),
					WindsensorModel(360,0,0),
					CompassModel(0,0,0,AccelerationModel(0,0,0) ),
					AnalogArduinoModel(0,0,0,0),
					0,
					0
				);
		heading = 180;

		twd = Utility::calculateTrueWindDirection(model,heading);

		REQUIRE(twd == Approx(180).epsilon(0.01));


	}
}
