#include "catch.hpp"
#include "windvanecontroller/WindVaneController.h"

TEST_CASE("testWindVaneController") {

	SECTION("setVaneAngle") {
		WindVaneController w;

		w.setVaneAngle(10, 10);
		REQUIRE(w.getVaneAngle() == 0);
		w.setVaneAngle(10, 20);
		REQUIRE(w.getVaneAngle() == 350);
		w.setVaneAngle(350, 20);
		REQUIRE(w.getVaneAngle() == 330);
	}
	SECTION("adjustVaneAngle") {
		WindVaneController w;

                w.setVaneAngle(10,10);
		REQUIRE(w.getVaneAngle() == 0);
                
                w.adjustAngle(10,20);
                
                REQUIRE(w.getVaneAngle() == 350);
                
                w.adjustAngle(315,45);
                
                REQUIRE(w.getVaneAngle() == 260);
                
                w.adjustAngle(45,315);
                
                REQUIRE(w.getVaneAngle() == 350);
                
                
                
		
	}

}
