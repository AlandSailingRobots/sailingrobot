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
			for (uint16_t i=testBearing + 1; i<testBearing + 90; ++i){
				TS_ASSERT_EQUALS(ballot.get(i), 0);
			}
			TS_ASSERT_LESS_THAN(0, ballot.get(testBearing+ 90));
			for (uint16_t i=testBearing + 91; i<360; ++i){
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
				i<testBearing + 90 - avoidanceBearingRange; ++i){
				TS_ASSERT_EQUALS(ballot.get(i), 0);
			}
			TS_ASSERT_LESS_THAN(0, ballot.get(testBearing+ 90));
			compareVote = ballot.get(testBearing);
			for (uint16_t i=testBearing + 90 - avoidanceBearingRange + 1; i<testBearing + 90 + 1; ++i){
				TS_ASSERT_LESS_THAN(compareVote, ballot.get(i));
				compareVote = ballot.get(i);
			}
			for (uint16_t i=testBearing + 90 + 1; i < testBearing + 90 + avoidanceBearingRange; ++i){
				TS_ASSERT_LESS_THAN(ballot.get(i), compareVote);
				compareVote = ballot.get(i);
			}	
			for (uint16_t i=testBearing + 90 + avoidanceBearingRange; i < 360; ++i){
				TS_ASSERT_EQUALS(ballot.get(i), 0);
			}	
		}
 		void testVisualAvoidance(){
 			CollidableMgr collidableManager;
			ProximityVoter proximityVoter(maxVotes, weight, collidableManager);
			std::map<int16_t, uint16_t> bearingToRelativeObstacleDistance;
			for (int i=-15; i<-10; ++i){
				bearingToRelativeObstacleDistance[i] = 100;
			}
			for (int i=-10; i<0; ++i){
				bearingToRelativeObstacleDistance[i] = 25;
			}
			for (int i=0; i<15; ++i){
				bearingToRelativeObstacleDistance[i] = 100;
			}
			collidableManager.addVisualField(bearingToRelativeObstacleDistance);
			proximityVoter.visualAvoidance();
			const ASRCourseBallot& ballot = proximityVoter.courseBallot;
			int minBearing = 0;
			int maxBearing = 0;
			int minVote = ballot.maxVotes();
			int maxVote = - minVote;
			for (int i=0; i<360; ++i){
				auto vote = ballot.get(i);
				if (vote > maxVote){
					maxVote = vote;
					maxBearing = i;
				}
				if (vote < minVote){
					minVote = vote;
					minBearing = i;
				}
			}
			TS_ASSERT_LESS_THAN(90 - 10, maxBearing);
			TS_ASSERT_LESS_THAN(maxBearing, 90);
			TS_ASSERT_LESS_THAN(360 - 10, minBearing); 
			TS_ASSERT_LESS_THAN(minBearing, 360); 
			TS_ASSERT_LESS_THAN(0, maxVote);
			TS_ASSERT_LESS_THAN(minVote, -ballot.maxVotes()* 0.9)
			for (int i=-15; i<-10; ++i){
				bearingToRelativeObstacleDistance[i] = 25;
			}
			for (int i=-10; i<15; ++i){
				bearingToRelativeObstacleDistance[i] = 100;
			}
			collidableManager.addVisualField(bearingToRelativeObstacleDistance);
			proximityVoter.visualAvoidance();
			minBearing = 0;
			maxBearing = 0;
			minVote = ballot.maxVotes();
			maxVote = - minVote;
			for (int i=0; i<360; ++i){
				auto vote = ballot.get(i);
				if (vote > maxVote){
					maxVote = vote;
					maxBearing = i;
				}
				if (vote < minVote){
					minVote = vote;
					minBearing = i;
				}
			}
			TS_ASSERT_LESS_THAN(0, maxBearing);
			TS_ASSERT_LESS_THAN(maxBearing, 15);
			TS_ASSERT_LESS_THAN(360 - 20, minBearing); 
			TS_ASSERT_LESS_THAN(minBearing, 360 - 10); 
			TS_ASSERT_LESS_THAN(minVote, -ballot.maxVotes()* 0.9)
		}

};