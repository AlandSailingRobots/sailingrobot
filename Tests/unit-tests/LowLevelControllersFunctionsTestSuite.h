/****************************************************************************************
*
* File:
* 		StateEstimationNodeSuite.h
*
* Purpose:
*		A set of unit tests for checking if the StateEstimationNode works as intended
*
* Developer Notes:
*
*
***************************************************************************************/

#pragma once

#include "Math/Utility.h"
#include "../cxxtest/cxxtest/TestSuite.h"
#include "SystemServices/CourseRegulator.h"
#include "SystemServices/WingsailControl.h"
#include "waypointrouting/Commands.h"


// For std::this_thread
#include <chrono>
#include <thread>

#define TEST_COUNT 3


class LowLevelControllersFunctionsTestSuite : public CxxTest::TestSuite
{
public:

  CourseRegulator* cregulator;

  void setUp()
  {
  }

  void tearDown()
  {
  }

  void test_CourseRegulatorRudderCalculation(){
    double maxRudderAngle = 30;
    double maxCourseAngleDiff = 60;
    double vesselCourse = 2.3;
    double courseToSteer = 3.1;

    CourseRegulator cregulator(maxRudderAngle, maxCourseAngleDiff);

    Commands commands;

    cregulator.setVesselCourse(vesselCourse);
    cregulator.setCourseToSteer(courseToSteer);
    double commandCalc = commands.rudderCommand(courseToSteer, vesselCourse, maxCourseAngleDiff);
    double regulatorCalc = cregulator.calculateRudderAngle();

    TS_ASSERT_DELTA(commandCalc * maxRudderAngle, regulatorCalc, 1e-3);
  }

  void test_WingsailControllerServoAngleCalculation(){
    int minServoAngleDiff = 5;
    int maxServoSailAngle = 10;
    double twd = 1.2;
    double heading = 2.6;

    WingsailControl wscontrol(minServoAngleDiff,maxServoSailAngle);
    wscontrol.setTrueWindDirection(twd);
    wscontrol.setVesselHeading(heading);

    float hwd = Utility::limitAngleRange(heading - twd);

    if(hwd > 180){
        TS_ASSERT_DELTA(wscontrol.calculateServoAngle(),  maxServoSailAngle, 1e-3);
    } else {
        TS_ASSERT_DELTA(wscontrol.calculateServoAngle(), -maxServoSailAngle, 1e-3);
    }
  }
};
