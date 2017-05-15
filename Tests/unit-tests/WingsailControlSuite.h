#include "Math/Utility.h"
#include "../cxxtest/cxxtest/TestSuite.h"
#include "SystemServices/WingsailControl.h"

class WingsailControlSuite : public CxxTest::TestSuite {
public:
    void setup() {

    }

    void tearDown() {

    }

    void test_ServoAngleCalculation() {
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

private:

};