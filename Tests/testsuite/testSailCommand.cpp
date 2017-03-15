
#include "catch.hpp"
#include "sailcommand/SailCommand.h"


TEST_CASE("testSailCommand")
{
	SECTION("Test SailCommand")
	{
		SailCommand sc;
		sc.setCommandValues(4000, 5000);
		REQUIRE(sc.getCommand(-3) == 4000);
		REQUIRE(sc.getCommand(0) == 4000);
		REQUIRE(sc.getCommand(0.1) == 4100);
		REQUIRE(sc.getCommand(0.5) == 4500);
		REQUIRE(sc.getCommand(0.8) == 4800);
		REQUIRE(sc.getCommand(1) == 5000);
		REQUIRE(sc.getCommand(2.34) == 5000);
	}
}
