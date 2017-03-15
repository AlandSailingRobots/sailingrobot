#include <iostream>
#include <math.h>
#include "catch.hpp"
#include "coursecalculation/CourseMath.h"
#include "models/PositionModel.h"

TEST_CASE("testCourseMath")
{
	SECTION("Test calculateDTW with 3 different positions for waypoint and boat")
	{
		const int noOfDTW = 3;
		double boatLats [3] 		= {60.836881, 60.073063, 59.539888 };
		double boatLongs [3] 		= {19.143219, 22.185974, 19.894409 };
		double waypointLats [3] 	= {60.103333, 60.103172, 60.103362 };
		double waypointLongs [3] 	= {19.928306, 19.92834, 19.928601 };

		/* Resulting distance between boat and waypont in meters.
		 * Taken from http://www.movable-type.co.uk/scripts/latlong.html 
		 */
		const double result_distance[3] = { 92218.1699654549, 125222.8858237675, 62684.5898278684 };

		CourseMath cm;
		for(int i = 0; i < noOfDTW; i++) {
			double dtw = cm.calculateDTW(PositionModel(boatLats[i], boatLongs[i]),
							PositionModel(waypointLats[i], waypointLongs[i]));
			REQUIRE(result_distance[i] == Approx(dtw));
		}
	}


	SECTION("Test calculateBTW with 3 different positions for waypoint and boat")
	{
		const int noOfBTW = 3;
		double boatLats [3] 		= {60.836881, 60.073063, 59.539888 };
		double boatLongs [3] 		= {19.143219, 22.185974, 19.894409 };
		double waypointLats [3] 	= {60.103333, 60.103172, 60.103362 };
		double waypointLongs [3] 	= {19.928306, 19.92834, 19.928601 };

		/* Resulting bearing between boat and waypont.
		 * Taken from http://www.movable-type.co.uk/scripts/latlong.html 
		 */
		const double result_bearing[3] = { 151.845945039753, 272.51025491197, 1.732425848657 };

		CourseMath cm;
		for(int i = 0; i < noOfBTW; i++) {
			double btw = cm.calculateBTW(PositionModel(boatLats[i], boatLongs[i]),
							PositionModel(waypointLats[i], waypointLongs[i]));
			REQUIRE(result_bearing[i] == Approx(btw));
		}
	}
}