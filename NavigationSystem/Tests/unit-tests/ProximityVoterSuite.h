/****************************************************************************************
 *
 * File:
 * 		ProximityVoterSuite.h
 *
 * Purpose:
 *
 * License:
 *      This file is subject to the terms and conditions defined in the file 
 *      'LICENSE.txt', which is part of this source code package.
 *
 * Developer Notes:
 *
 ***************************************************************************************/

 #pragma once

 #include "Tests/cxxtest/cxxtest/TestSuite.h"
 #include "Navigation/LocalNavigationModule/Voters/ProximityVoter.h"
 #include "WorldState/CollidableMgr/CollidableMgr.h"


 class ProximityVoterSuite : public CxxTest::TestSuite {
	public:
		const int16_t maxVotes = 100;
		const int16_t weight = 1;

		void testAvoidOutsideVisualField(){
			CollidableMgr collidableManager;
			int bearingLowLimit = -10;
			int bearingHighLimit = 15;
			ProximityVoter proximityVoter(maxVotes, weight, collidableManager);
			proximityVoter.avoidOutsideVisualField(bearingLowLimit, bearingHighLimit);
			const ASRCourseBallot& ballot = proximityVoter.courseBallot; 
			for (uint16_t i=360 + bearingLowLimit; i<360; ++i){
				TS_ASSERT_EQUALS(ballot.get(i), 0);
			}
			for (uint16_t i=0; i<bearingHighLimit; ++i){
				TS_ASSERT_EQUALS(ballot.get(i), 0);
			}
			for (uint16_t i=bearingHighLimit; i<360 + bearingLowLimit; ++i){
				TS_ASSERT_LESS_THAN(ballot.get(i), 0);
				// This depends on config that may change
//				TS_ASSERT_EQUALS(ballot.get(i), -50);
			}
		}
		void testBearingAvoidanceSingleDir(){
			CollidableMgr collidableManager;
			ProximityVoter proximityVoter(maxVotes, weight, collidableManager);
			int testBearing = 37;
			auto voteAdjust = 70;
			proximityVoter.bearingAvoidanceSingleDir(testBearing, voteAdjust);
			const ASRCourseBallot& ballot = proximityVoter.courseBallot; 
			for (uint16_t i=0; i<testBearing; ++i){
				TS_ASSERT_EQUALS(ballot.get(i), 0);
			}
			TS_ASSERT_EQUALS(ballot.get(testBearing), -voteAdjust);
			for (uint16_t i=testBearing + 1; i<testBearing + 180; ++i){
				TS_ASSERT_EQUALS(ballot.get(i), 0);
			}
			TS_ASSERT_LESS_THAN(0, ballot.get(testBearing+ 180));
			for (uint16_t i=testBearing + 181; i<360; ++i){
				TS_ASSERT_EQUALS(ballot.get(i), 0);
			}	

		}
		void testBearingAvoidanceSmoothed(){
			CollidableMgr collidableManager;
			ProximityVoter proximityVoter(maxVotes, weight, collidableManager);
			int testBearing = 37;
    		const uint16_t avoidanceBearingRange = 10;  // copied from ProximityVoter
			auto relativeObstacleDistance = 5;  // This must be a small number for the strict less than checks to pass
			proximityVoter.bearingAvoidanceSmoothed(testBearing, relativeObstacleDistance);
			const ASRCourseBallot& ballot = proximityVoter.courseBallot; 
			for (uint16_t i=0; i<testBearing - avoidanceBearingRange; ++i){
				TS_ASSERT_EQUALS(ballot.get(i), 0);
			}
			auto compareVote = 0;
			for (uint16_t i = testBearing - avoidanceBearingRange + 1; i<testBearing; ++i){
				TS_ASSERT_LESS_THAN(ballot.get(i), compareVote);
				compareVote = ballot.get(i);
			}
			TS_ASSERT_EQUALS(ballot.get(testBearing), -(100 - relativeObstacleDistance)*2.0/avoidanceBearingRange);
			compareVote = ballot.get(testBearing);
			for (uint16_t i=testBearing + 1; i<testBearing + avoidanceBearingRange; ++i){
				TS_ASSERT_LESS_THAN(compareVote, ballot.get(i));
				compareVote = ballot.get(i);
			}
			for (uint16_t i=testBearing + avoidanceBearingRange; 
				i<testBearing + 180 - avoidanceBearingRange; ++i){
				TS_ASSERT_EQUALS(ballot.get(i), 0);
			}
			TS_ASSERT_LESS_THAN(0, ballot.get(testBearing+ 180));
			compareVote = ballot.get(testBearing);
			for (uint16_t i=testBearing + 180 - avoidanceBearingRange + 1; i<testBearing + 180 + 1; ++i){
				TS_ASSERT_LESS_THAN(compareVote, ballot.get(i));
				compareVote = ballot.get(i);
			}
			for (uint16_t i=testBearing + 180 + 1; i < testBearing + 180 + avoidanceBearingRange; ++i){
				TS_ASSERT_LESS_THAN(ballot.get(i), compareVote);
				compareVote = ballot.get(i);
			}	
			for (uint16_t i=testBearing + 180 + avoidanceBearingRange; i < 360; ++i){
				TS_ASSERT_EQUALS(ballot.get(i), 0);
			}	
		}
 };