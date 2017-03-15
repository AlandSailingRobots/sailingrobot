
#include "catch.hpp"
#include <iostream>
#include "Compass/Compass.h"
#include "Compass/HMC6343.h"
#include "Compass/MockCompass.h"

using namespace std;

TEST_CASE("testCompass")
{

	SECTION("CompassTest real compass init") {
		HMC6343 c;
		Compass* compass = &c;
		REQUIRE(compass->init());
	}
	
	SECTION("CompassTest mock compass init") {
		MockCompass c;
		Compass* compass = &c;
		REQUIRE(compass->init());
	} 
	
	SECTION("CompassTest mock compass values") {
		MockCompass c;
		Compass* compass = &c;
		
                REQUIRE(compass->getHeading() == 100);
                REQUIRE(compass->getPitch() == 200);
                REQUIRE(compass->getRoll() == 300);
                REQUIRE(compass->getAccelX() == 5);
                REQUIRE(compass->getAccelY() == 5);
                REQUIRE(compass->getAccelZ() == 5);
	} 
}
