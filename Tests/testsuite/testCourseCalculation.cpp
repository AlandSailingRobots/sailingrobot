#include <iostream>
#include "catch.hpp"
#include "coursecalculation/CourseCalculation.h"
#include "../models/PositionModel.h"
#include "../models/WaypointModel.h"

TEST_CASE("testCourseCalculation")
{
	SECTION("Test CTS is BTW unless course is within TACK_ANGLE degrees from TWD, if not tack")
	{
		CourseCalculation cc;

		double boatLats [3] 		= {60.836881, 60.073063, 59.539888 };
		double boatLongs [3] 		= {19.143219, 22.185974, 19.894409};
		double waypointLats [3] 	= {60.103333, 60.103172, 60.103362};
		double waypointLongs [3] 	= {19.928306, 19.92834, 19.928601};

		const int tack_angle = 45;
		const int sector_angle = 5;
		const int TWD = 270;

		cc.setTackAngle(tack_angle);
		cc.setSectorAngle(sector_angle);

		const double min_within_value = TWD - tack_angle;
		const double max_within_value = TWD + tack_angle;


		for(int i = 0; i < 3; i++) {
			
			cc.calculateCourseToSteer(PositionModel(boatLats[i], boatLongs[i]),
				WaypointModel(PositionModel(waypointLats[i], waypointLongs[i]), 10, "1", 6), TWD);

			/* If desired course is within TACK_ANGLE +/- TWD then we should be tacking.*/
			if(cc.getCTS() >= min_within_value && cc.getCTS() <= max_within_value) {
				/* Now we should be tacking. */
				REQUIRE(cc.getTack());

				/* Course should now be equal to TWD +/- TACK_ANGLE. */
				bool course_OK = false;

				if(cc.getCTS() == min_within_value || cc.getCTS() == max_within_value) {
					course_OK = true;	
				}
				
				REQUIRE(course_OK);						
			}
			else {
				/* courseToSteer should be equal to BearingToWayPoint.
				   And we should NOT be tacking. */
				REQUIRE(cc.getCTS() == cc.getBTW());
				REQUIRE_FALSE(cc.getTack());
			}				
		}
	}
}


TEST_CASE(" testCourseCalculationToSteerTrackdirection")
{
	//waypoint straight east of the boat
	double boatLat = 60.836881, boatLong = 19.143219;
	double waypointLat = 60.836881, waypointLong = 20.943219;
	double tackAngle = 45;
	double sectorAngle = 5;
	CourseCalculation cc;
	cc.setTackAngle(tackAngle);
	cc.setSectorAngle(sectorAngle);

	SECTION("boat is outside tacksector, clockwise of winddirection")
	{
		double TWD = 70;
		cc.calculateCourseToSteer(PositionModel(boatLat, boatLong),
			WaypointModel(PositionModel(waypointLat, waypointLong), 10, "1", 6), TWD);

		REQUIRE(cc.getCTS() == (TWD + tackAngle));
	}

	SECTION("boat is inside tacksector, clockwise of winddirection")
	{
		double TWD = 86;
		cc.calculateCourseToSteer(PositionModel(boatLat, boatLong),
			WaypointModel(PositionModel(waypointLat, waypointLong), 10, "1", 6), TWD);

		REQUIRE(cc.getCTS() == (TWD + tackAngle));
	}

	SECTION("boat is inside tacksector, counterclockwise of winddirection")
	{
		double TWD = 92;
		cc.calculateCourseToSteer(PositionModel(boatLat, boatLong),
			WaypointModel(PositionModel(waypointLat, waypointLong), 10, "1", 6), TWD);

		REQUIRE(cc.getCTS() == (TWD - tackAngle));
	}

	SECTION("boat is outside tacksector, counterclockwise of winddirection")
	{
		double TWD = 100;
		cc.calculateCourseToSteer(PositionModel(boatLat, boatLong),
			WaypointModel(PositionModel(waypointLat, waypointLong), 10, "1", 6), TWD);

		REQUIRE(cc.getCTS() == (TWD - tackAngle));
	}
}


TEST_CASE("testCourseCalculationToSteerSequence")
{
	double boatLat = 60.104320, boatLong = 19.918823;
	double waypointLat = 60.107597, waypointLong = 19.922965;
	double tackAngle = 45;
	double sectorAngle = 5;
	CourseCalculation cc;
	cc.setTackAngle(tackAngle);
	cc.setSectorAngle(sectorAngle);

	SECTION("notack, 2xsbtack, 2xptack, 2xsbtack, 2xptack, 2xsbtack, notack")
	{
		double TWD = 225; 
		cc.calculateCourseToSteer(PositionModel(boatLat, boatLong),
			WaypointModel(PositionModel(waypointLat, waypointLong), 25, "1", 6), TWD);
		REQUIRE_FALSE(cc.getTack());
		REQUIRE(cc.getCTS() == cc.getBTW());

		

		TWD = 20;
		cc.calculateCourseToSteer(PositionModel(boatLat, boatLong),
			WaypointModel(PositionModel(waypointLat, waypointLong), 25, "1", 6), TWD);
		REQUIRE(cc.getTack());
		REQUIRE(cc.getGoingStarboard());
		REQUIRE(cc.getCTS() == (TWD + tackAngle));

		boatLat = 60.10531;
		boatLong = 19.922123;
		cc.calculateCourseToSteer(PositionModel(boatLat, boatLong),
			WaypointModel(PositionModel(waypointLat, waypointLong), 25, "1", 6), TWD);
		REQUIRE(cc.getTack());
		REQUIRE(cc.getGoingStarboard());
		REQUIRE(cc.getCTS() == (TWD + tackAngle));



		boatLat = 60.10534;
		boatLong = 19.922223;
		cc.calculateCourseToSteer(PositionModel(boatLat, boatLong),
			WaypointModel(PositionModel(waypointLat, waypointLong), 25, "1", 6), TWD);
		REQUIRE(cc.getTack());
		REQUIRE_FALSE(cc.getGoingStarboard());
		REQUIRE(cc.getCTS() == (360 + (TWD - tackAngle)));

		boatLat = 60.10576;
		boatLong = 19.920823;
		cc.calculateCourseToSteer(PositionModel(boatLat, boatLong),
			WaypointModel(PositionModel(waypointLat, waypointLong), 25, "1", 6), TWD);
		REQUIRE(cc.getTack());
		REQUIRE_FALSE(cc.getGoingStarboard());
		REQUIRE(cc.getCTS() == (360 + (TWD - tackAngle)));



		boatLat = 60.10579;
		boatLong = 19.920723;
		cc.calculateCourseToSteer(PositionModel(boatLat, boatLong),
			WaypointModel(PositionModel(waypointLat, waypointLong), 25, "1", 6), TWD);
		REQUIRE(cc.getTack());
		REQUIRE(cc.getGoingStarboard());
		REQUIRE(cc.getCTS() == (TWD + tackAngle));

		boatLat = 60.10639;
		boatLong = 19.922723;
		cc.calculateCourseToSteer(PositionModel(boatLat, boatLong),
			WaypointModel(PositionModel(waypointLat, waypointLong), 25, "1", 6), TWD);
		REQUIRE(cc.getTack());
		REQUIRE(cc.getGoingStarboard());
		REQUIRE(cc.getCTS() == (TWD + tackAngle));

	

		boatLat = 60.10642;
		boatLong = 19.922823;
		cc.calculateCourseToSteer(PositionModel(boatLat, boatLong),
			WaypointModel(PositionModel(waypointLat, waypointLong), 25, "1", 6), TWD);
		REQUIRE(cc.getTack());
		REQUIRE_FALSE(cc.getGoingStarboard());
		REQUIRE(cc.getCTS() == (360 + (TWD - tackAngle)));

		boatLat = 60.10675;
		boatLong = 19.921723;
		cc.calculateCourseToSteer(PositionModel(boatLat, boatLong),
			WaypointModel(PositionModel(waypointLat, waypointLong), 25, "1", 6), TWD);
		REQUIRE(cc.getTack());
		REQUIRE_FALSE(cc.getGoingStarboard());
		REQUIRE(cc.getCTS() == (360 + (TWD - tackAngle)));



		boatLat = 60.10678;
		boatLong = 19.921623;
		cc.calculateCourseToSteer(PositionModel(boatLat, boatLong),
			WaypointModel(PositionModel(waypointLat, waypointLong), 25, "1", 6), TWD);
		REQUIRE(cc.getTack());
		REQUIRE(cc.getGoingStarboard());
		REQUIRE(cc.getCTS() == (TWD + tackAngle));

		boatLat = 60.10678;
		boatLong = 19.921623;
		cc.calculateCourseToSteer(PositionModel(boatLat, boatLong),
			WaypointModel(PositionModel(waypointLat, waypointLong), 25, "1", 6), TWD);
		REQUIRE(cc.getTack());
		REQUIRE(cc.getGoingStarboard());
		REQUIRE(cc.getCTS() == (TWD + tackAngle));



		boatLat = 60.10729;
		boatLong = 19.923323;
		cc.calculateCourseToSteer(PositionModel(boatLat, boatLong),
			WaypointModel(PositionModel(waypointLat, waypointLong), 25, "1", 6), TWD);
		REQUIRE_FALSE(cc.getTack());
		REQUIRE(cc.getCTS() == cc.getBTW());


	}
}
