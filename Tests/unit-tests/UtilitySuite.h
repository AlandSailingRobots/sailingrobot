/****************************************************************************************
 *
 * File:
 * 		UtilitySuite.h
 *
 * Purpose:
 *		A set of unit tests for the utility functions
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/

#pragma once


#include "../cxxtest/cxxtest/TestSuite.h"
#include "Math/Utility.h"
#include "SystemServices/Timer.h"
#include <stdint.h> // uint8_t
#include "../testsuite/catch.hpp"


class UtilitySuite : public CxxTest::TestSuite {
public:
	void setUp() { }

	void tearDown() { }

	void test_CombineBytes()
	{
		uint8_t MSB = 1; // = 256
		uint8_t LSB = 4;
		int combined = Utility::combineBytes(MSB, LSB);
		TS_ASSERT_EQUALS(combined, 260);
	}

	void test_MedianEven()
	{
		float f[] = {1.0, 2.0, 20.0, 0.0, 5.0, 5.0};
		std::vector<float> v (f, f + sizeof(f) / sizeof(float) );
		float middle = Utility::getMedianValue(v);
		TS_ASSERT_EQUALS(middle, 3.5);
	}

	void test_MedianOdd()
	{
		float f[] = {1.0, 2.0, 20.0, 5.0, 5.0};
		std::vector<float> v (f, f + sizeof(f) / sizeof(float) );
		float middle = Utility::getMedianValue(v);
		TS_ASSERT_EQUALS(middle, 5);
	}

	void test_MedianEmptyVector()
	{
		std::vector<float> v;
		float middle = Utility::getMedianValue(v);
		TS_ASSERT_EQUALS(middle, 0);
	}

	void test_Mean()
	{
		float f[] = {1,2,3};
		std::vector<float> values (f, f + sizeof(f) / sizeof(float) );
		TS_ASSERT_EQUALS(Utility::mean(values), 2);

		float f2[] = {1,2};
		std::vector<float> values2 (f2, f2 + sizeof(f2) / sizeof(float) );
		TS_ASSERT_EQUALS(Utility::mean(values2), 1.5);
	}

	void test_MeanEmptyVector()
	{
		std::vector<float> v;
		TS_ASSERT_EQUALS(Utility::mean(v), 0);
	}

	void test_MeanOfAngles()
	{
		float f[] = {0,180, 90};
		std::vector<float> values (f, f + sizeof(f) / sizeof(float) );
		TS_ASSERT_EQUALS(Utility::meanOfAngles(values), 90);
	}

	void test_MeanOfAnglesAround0Degrees()
	{
		float f[] = {330,30};
		std::vector<float> values (f, f + sizeof(f) / sizeof(float) );
		TS_ASSERT_EQUALS(Utility::meanOfAngles(values), 0);
	}

	void test_MeanOfAnglesEmptyVector()
	{
		std::vector<float> v;
		TS_ASSERT_EQUALS(Utility::meanOfAngles(v), 0);
	}

	void test_MeanOfAnglesGreaterThan360()
	{
		float f[] = {370};
		std::vector<float> values (f, f + sizeof(f) / sizeof(float) );
		TS_ASSERT_EQUALS(Utility::meanOfAngles(values), 10);
	}

	void test_MeanOfAnglesLessThan0()
	{
		float f[] = {-60};
		std::vector<float> values (f, f + sizeof(f) / sizeof(float) );
		TS_ASSERT_EQUALS(Utility::meanOfAngles(values), 300);
	}

	void test_PolarToCartesianCoordinates()
	{
		float x, y;
		Utility::polarToCartesian(90, x, y);
		TS_ASSERT_DELTA(x, 0, 1e-7);
		TS_ASSERT_DELTA(y, 1, 1e-7);
	}

	void test_AngleInSector()
	{
		TS_ASSERT(Utility::isAngleInSector(10, 0, 30));
		TS_ASSERT(Utility::isAngleInSector(10, 0, 184));
		TS_ASSERT(Utility::isAngleInSector(10, 340, 40));
		TS_ASSERT(Utility::isAngleInSector(350, 340, 40));
		TS_ASSERT(Utility::isAngleInSector(10, -40, 40));
		TS_ASSERT(Utility::isAngleInSector(10, 0, 375));
		TS_ASSERT(Utility::isAngleInSector(10, 0, 10));
		TS_ASSERT(not Utility::isAngleInSector(10, 40, 340));
		TS_ASSERT(not Utility::isAngleInSector(-10, 0, 30));
	}

	void test_DifferenceBetweenAngles()
	{
		TS_ASSERT_EQUALS(Utility::angleDifference(359, 0), 1);
		TS_ASSERT_EQUALS(Utility::angleDifference(0, 359), 1);
		TS_ASSERT_EQUALS(Utility::angleDifference(0, 190), 170);
		TS_ASSERT_EQUALS(Utility::angleDifference(500, 1070), 150);
		TS_ASSERT_EQUALS(Utility::angleDifference(1070, 500), 150);
		TS_ASSERT_EQUALS(Utility::angleDifference(-10, 10), 20);
		TS_ASSERT_EQUALS(Utility::angleDifference(-1070, -500), 150);
	}

	void test_LimiteAngleRange()
	{
		TS_ASSERT_EQUALS(Utility::limitAngleRange(1), 1);
		TS_ASSERT_EQUALS(Utility::limitAngleRange(359), 359);
		TS_ASSERT_EQUALS(Utility::limitAngleRange(360), 0);
		TS_ASSERT_EQUALS(Utility::limitAngleRange(0), 0);
		TS_ASSERT_EQUALS(Utility::limitAngleRange(-1), 359);
		TS_ASSERT_EQUALS(Utility::limitAngleRange(620), 260);
		TS_ASSERT_EQUALS(Utility::limitAngleRange(2050), 250);
		TS_ASSERT_EQUALS(Utility::limitAngleRange(-2050), 110);
	}

	void test_DegreeRadianConversion()
	{
		TS_ASSERT_EQUALS(Utility::degreeToRadian(0.0), 0.0);
		TS_ASSERT_EQUALS(Utility::degreeToRadian(180.0), Approx(M_PI));
		TS_ASSERT_EQUALS(Utility::degreeToRadian(90.0), Approx(M_PI/2));
		TS_ASSERT_EQUALS(Utility::degreeToRadian(360.0), Approx(M_PI*2));
	}

	void test_RadianToDegreeConversion()
	{
		TS_ASSERT_EQUALS(Utility::radianToDegree(0.0), 0.0);
		TS_ASSERT_EQUALS(Utility::radianToDegree(1), Approx(57.2957795));
		TS_ASSERT_EQUALS(Utility::radianToDegree(M_PI) , Approx(180.0));
	}

	void test_AddingDeclinationToHeading()
	{
		TS_ASSERT_EQUALS(Utility::addDeclinationToHeading(45, 6), 51);
		TS_ASSERT_EQUALS(Utility::addDeclinationToHeading(0, -6), 354);
		TS_ASSERT_EQUALS(Utility::addDeclinationToHeading(355, 6), 1);
	}

	void test_TrueWindDirection()
	{
		// std::vector<float> twdBuffer; 				//ELOUAN PLS HELP
		// unsigned int twdBufferMaxSize = 100;
		// TS_ASSERT_EQUALS(Utility::getTrueWindDirection(170, 5, 2, 100, twdBuffer, twdBufferMaxSize), 272.8526f);
		// TS_ASSERT_EQUALS(Utility::getTrueWindDirection(171, 5, 2.1, 100.1, twdBuffer, twdBufferMaxSize), 273.2557f)
		// TS_ASSERT_EQUALS(Utility::getTrueWindDirection(165, 5, 2, 100, twdBuffer, twdBufferMaxSize), 271.9276f);
		TS_FAIL("Not implemented!");
	}
};
