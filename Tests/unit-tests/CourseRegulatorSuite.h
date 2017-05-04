#include "Math/Utility.h"
#include "../cxxtest/cxxtest/TestSuite.h"
#include "SystemServices/CourseRegulator.h"
#include "waypointrouting/Commands.h"

class CourseRegulatorSuite : public CxxTest::TestSuite {
public:
    void setup() {

    }

    void tearDown() {

    }

    // Test assumes that Commands calculation is correct.

    void test_RudderCalculation() {

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

}; 