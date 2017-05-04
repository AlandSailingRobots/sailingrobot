#include "Math/Utility.h"
#include "../cxxtest/cxxtest/TestSuite.h"
#include "SystemServices/WingsailControl.h"

class WingsailControlSuite : public CxxTest::TestSuite {
public:
    void setup() {

    }

    void tearDown() {

    }

    void testServoAngleCalculation() {
        int minServoAngleDiff = 5;
        int maxServoSailAngle = 10;
        double twd = 1.2;
        double heading = 2.6;

        WingsailControl wscontrol(minServoAngleDiff,maxServoSailAngle);
        wscontrol.setTrueWindDirection(twd);
        wscontrol.setVesselHeading(heading);
        
    }

private:

};