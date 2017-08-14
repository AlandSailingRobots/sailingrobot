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
 *							11.4.17 JM
 *
 *	Functions that have tests:		Functions that does not have tests:
 *
 *	combineBytes 					combineBytesSigned
 *	getMedianValue 					IsOutOfRange
 *	mean							fixAngles
 *	meanOfAngles					directionAdjustedSpeed
 *	sgn 							calculateSignedDistanceToLine
 *	polarToCartesian 				calculateWaypointsOrthogonalLine
 *	isAngleInSector 				calculateTrueWindSpeed
 *	angleDifference 				calculateApparentWind
 *	limitAngleRange 				getApparentWindSpeed
 *	limitRadianAngleRange			getApparentWindDirection
 *	degreeToRadian
 *	radianToDegree
 *	headingDifference
 *	wrapAngle
 *	addDeclinationToHeading
 *
 ***************************************************************************************/

#pragma once


#include "../cxxtest/cxxtest/TestSuite.h"
#include "Math/Utility.h"
#include "SystemServices/Timer.h"
#include <stdint.h> // uint8_t

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

	void test_Sgn()
	{
		double a1 = -34.6, a2 = 0, a3 = 234.567;
		int b1 = Utility::sgn(a1);
		int b2 = Utility::sgn(a2);
		int b3 = Utility::sgn(a3);

		TS_ASSERT_EQUALS(b1, -1);
		TS_ASSERT_EQUALS(b2, 0);
		TS_ASSERT_EQUALS(b3, 1);
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

	void test_LimitAngleRange()
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

	void test_LimitRadianAngleRange()
	{

		TS_ASSERT_DELTA(Utility::limitRadianAngleRange(0),0, 1e-7);
		TS_ASSERT_DELTA(Utility::limitRadianAngleRange(M_PI),M_PI, 1e-7);
		TS_ASSERT_DELTA(Utility::limitRadianAngleRange(M_PI*2),0, 1e-7);
		TS_ASSERT_DELTA(Utility::limitRadianAngleRange(M_PI*3),M_PI, 1e-7);
		TS_ASSERT_DELTA(Utility::limitRadianAngleRange(M_PI*-3),M_PI, 1e-7);
	}

	void test_LinearAngle()
	{
		TS_ASSERT_DELTA(Utility::linearFunctionBetweenAngle(0.6, 0, 1, 90, 100), 96, 1e-7);
		TS_ASSERT_DELTA(Utility::linearFunctionBetweenAngle(0.6, 0, 1, 100, 90), 94, 1e-7);
		TS_ASSERT_DELTA(Utility::linearFunctionBetweenAngle(0, 0, 1, 90, 100), 90, 1e-7);
		TS_ASSERT_DELTA(Utility::linearFunctionBetweenAngle(1, 0, 1, 90, 100), 100, 1e-7);
		TS_ASSERT_DELTA(Utility::linearFunctionBetweenAngle(0.5, 0, 1, 10, 350), 0, 1e-7);
		TS_ASSERT_DELTA(Utility::linearFunctionBetweenAngle(0.5, 0, 1, 350, 10), 0, 1e-7);
		TS_ASSERT_DELTA(Utility::linearFunctionBetweenAngle(2, 1, 3, 10, -20), 355, 1e-7);
		TS_ASSERT_DELTA(Utility::linearFunctionBetweenAngle(2, 1, 3, -20, 10), 355, 1e-7);
		TS_ASSERT_DELTA(Utility::linearFunctionBetweenAngle(0.5, 0, 1, 350, 380), 5, 1e-7);
		TS_ASSERT_DELTA(Utility::linearFunctionBetweenAngle(0.5, 0, 1, 380, 350), 5, 1e-7);
		TS_ASSERT_DELTA(Utility::linearFunctionBetweenAngle(0.5, 0, 1, 180, 170), 175, 1e-7);
		TS_ASSERT_DELTA(Utility::linearFunctionBetweenAngle(0.5, 0, 1, 170, 180), 175, 1e-7);
		TS_ASSERT_DELTA(Utility::linearFunctionBetweenAngle(0, 0, 1, 0, 50), 0, 1e-7);
		TS_ASSERT_DELTA(Utility::linearFunctionBetweenAngle(0, 0, 1, 360, 50), 0, 1e-7);
	}


// Following tests have to be refactored with TS_ASSERT_DELTA
// Maël - 4/4/2017

	void test_DegreeRadianConversion()
	{
		TS_ASSERT_DELTA(Utility::degreeToRadian(0.0), 0.0, 1e-7);
		TS_ASSERT_DELTA(Utility::degreeToRadian(180.0), M_PI, 1e-7);
		TS_ASSERT_DELTA(Utility::degreeToRadian(90.0), M_PI/2, 1e-7);
		TS_ASSERT_DELTA(Utility::degreeToRadian(360.0), M_PI*2, 1e-7);
	}

	void test_RadianToDegreeConversion()
	{
		TS_ASSERT_DELTA(Utility::radianToDegree(0.0), 0.0, 1e-7);
		TS_ASSERT_DELTA(Utility::radianToDegree(1), 57.2957795, 1e-7);
		TS_ASSERT_DELTA(Utility::radianToDegree(M_PI) , 180.0, 1e-7);
	}

	void test_HeadingDifference()
	{
		TS_ASSERT_EQUALS(Utility::headingDifference(1,180), 179);
		TS_ASSERT_EQUALS(Utility::headingDifference(180,1), -179);
		TS_ASSERT_EQUALS(Utility::headingDifference(1,181), 180);
		TS_ASSERT_EQUALS(Utility::headingDifference(181,1), 180);
		TS_ASSERT_EQUALS(Utility::headingDifference(1,182), -179);
		TS_ASSERT_EQUALS(Utility::headingDifference(182,1), 179);
		TS_ASSERT_EQUALS(Utility::headingDifference(360,0), 0);
		TS_ASSERT_EQUALS(Utility::headingDifference(0,360), 0);
	}

	void test_WrapAngle()
	{
		TS_ASSERT_EQUALS(Utility::wrapAngle(183), 183);
		TS_ASSERT_EQUALS(Utility::wrapAngle(389), 29);
		TS_ASSERT_EQUALS(Utility::wrapAngle(-8913), 87);
	}

	void test_AddingDeclinationToHeading()
	{
		TS_ASSERT_DELTA(Utility::addDeclinationToHeading(45, 6), 51, 1e-7);
		TS_ASSERT_DELTA(Utility::addDeclinationToHeading(0, -6), 354, 1e-7);
		TS_ASSERT_DELTA(Utility::addDeclinationToHeading(355, 6), 1, 1e-7);
	}

	void test_TrueWindDirection()
	{
		std::vector<float> twdBuffer;
		unsigned int twdBufferMaxSize = 100;
		TS_ASSERT_DELTA(Utility::getTrueWindDirection(170, 5, 2, 100, twdBuffer, twdBufferMaxSize), 272.8526f, 1e-4);
		TS_ASSERT_DELTA(Utility::getTrueWindDirection(171, 5, 2.1, 100.1, twdBuffer, twdBufferMaxSize), 273.2557f, 1e-4)
		TS_ASSERT_DELTA(Utility::getTrueWindDirection(165, 5, 2, 100, twdBuffer, twdBufferMaxSize), 271.9276f, 1e-4);
	}

	void test_polarVerctorsAddition()
	{
		std::vector<double> v1(2), v2(2), v3(2);

		v1[0] = 3;		v1[1] = Utility::degreeToRadian(0);
		v2[0] = 4;		v2[1] = Utility::degreeToRadian(90);
		v3 = Utility::polarVerctorsAddition(v1, v2);
		TS_ASSERT_DELTA(v3[0], 5, 1e-7);
		TS_ASSERT_DELTA(v3[1], 0.92729, 1e-4);

		v1[0] = 3;		v1[1] = Utility::degreeToRadian(50);
		v2[0] = 4.5;	v2[1] = Utility::degreeToRadian(230);
		v3 = Utility::polarVerctorsAddition(v1, v2);
		TS_ASSERT_DELTA(v3[0], 1.5, 1e-7);
		TS_ASSERT_DELTA(v3[1], Utility::degreeToRadian(230), 1e-4);

		v1[0] = 3;		v1[1] = Utility::degreeToRadian(0);
		v2[0] = -3;		v2[1] = Utility::degreeToRadian(90);
		v3 = Utility::polarVerctorsAddition(v1, v2);
		TS_ASSERT_DELTA(v3[0], sqrt(18), 1e-7);
		TS_ASSERT_DELTA(v3[1], Utility::degreeToRadian(315), 1e-4);
	}
};
