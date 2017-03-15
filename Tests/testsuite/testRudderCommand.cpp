
#include "catch.hpp"
#include "ruddercommand/RudderCommand.h"


TEST_CASE("testRudderCommand")
{
	SECTION("rc test")
	{
		RudderCommand rc;
		rc.setCommandValues(7000, 5000);
		REQUIRE(rc.getCommand(-2) == 3000);
		REQUIRE(rc.getCommand(-1) == 3000);
		REQUIRE(rc.getCommand(-0.5) == 4000);
		REQUIRE(rc.getCommand(0) == 5000);
		REQUIRE(rc.getCommand(0.7) == 6400);
		REQUIRE(rc.getCommand(1) == 7000);
		REQUIRE(rc.getCommand(1.34) == 7000);
	}
}
