#include <iostream>
#include <cmath>
#include "Math/Utility.h"
#include "catch.hpp"
#include "utility/MockPosition.h"

using namespace std;

TEST_CASE("testPosition")
{
    SECTION("Position test mock Position init") {
        MockPosition position; 
        
        position.setHeading(10);
        
        REQUIRE(position.getHeading() == 10);
        
        REQUIRE(position.getModel().latitude == STARTLATPOSITION);
        REQUIRE(position.getModel().longitude == STARTLONGPOSITION);
        
        position.setCourseToSteer(5);
        
        REQUIRE(position.getHeading() == 9);
        
        position.updatePosition();
        
        double newLat = STARTLATPOSITION + 
                            std::cos(Utility::degreeToRadian(5) ) * LATUPDATE;
        double newLong = STARTLONGPOSITION + 
                            std::cos(Utility::degreeToRadian(5) ) * LONGUPDATE; 
        
        REQUIRE(position.getModel().latitude == newLat);
        REQUIRE(position.getModel().longitude == newLong);
        
    }
}
