/****************************************************************************************
 *
 * File:
 * 		ProximityVoter.cpp
 *
 * Purpose:
 *
 *
 * License:
 *      This file is subject to the terms and conditions defined in the file
 *      'LICENSE.txt', which is part of this source code package.
 *
 ***************************************************************************************/


#include "ProximityVoter.h"
#include "SystemServices/Logger.h"
#include "Math/CourseMath.h"
#include <vector>
#include "Math/Utility.h"


///----------------------------------------------------------------------------------
ProximityVoter::ProximityVoter( int16_t maxVotes, int16_t weight, CollidableMgr& collidableMgr )
    :ASRVoter( maxVotes, weight, "Proximity" ), collidableMgr(collidableMgr)
{

}

///----------------------------------------------------------------------------------
const ASRCourseBallot& ProximityVoter::vote( const BoatState_t& boatState )
{
    //test1(boatState, courseBallot, collidableMgr);
    courseBallot.clear();

    CollidableList<AISCollidable_t> aisContacts = collidableMgr.getAISContacts();

    float currClosest = 2016; // Default high value
    static float lifeTimeClosest = 2016;

    // AIS Contacts
    for(uint16_t i = 0; i < aisContacts.length(); i++)
    {
        AISCollidable_t collidable = aisContacts.next();

        float distance = aisAvoidance( boatState, collidable );

        if(distance < currClosest)
        {
            currClosest = distance;
        }

        if(distance < lifeTimeClosest)
        {
            lifeTimeClosest = distance;
        }
    }

    visualAvoidance(boatState.heading);

    auto maxVote = -courseBallot.maxVotes();
    auto minVote = courseBallot.maxVotes();
    auto maxBearing = 0;
    auto minBearing = 0; 
    for (int i=0; i<360; ++i){
        auto vote = courseBallot.get(i);
        if (vote > maxVote){
            maxVote = vote;
            maxBearing = i;
        }
        if (vote < minVote){
            minVote = vote;
            minBearing = i;
        }
    }
    Logger::info("Max vote: %d Min vote: %d", maxVote, minVote);
    Logger::info("Max bearing: %d Min bearing: %d", maxBearing, minBearing);

 
    //Logger::info("Lifetime Closest: %f Closest: %f", lifeTimeClosest, currClosest);

    // Need to take tacking into account, otherwise a upwind course might be chosen
    std::vector<float> trueWindBuffer;
    uint16_t twd = Utility::getTrueWindDirection(boatState.windDir, boatState.windSpeed,
                boatState.speed, boatState.heading, trueWindBuffer, 1);
    Logger::info("True wind dir: %d", twd);

    // Set 0 to courses into the no go zone.
    for( int i = 0; i < 45; i+= ASRCourseBallot::COURSE_RESOLUTION )
    {
        courseBallot.set( twd + i, 0 );
        courseBallot.set( twd - i, 0 );
    }

    return courseBallot;
}

void ProximityVoter::visualAvoidance(uint16_t heading){
    VisualField_t visualField = collidableMgr.getVisualField();
    if (visualField.bearingToRelativeObstacleDistance.empty()){
        return;
    }
    auto lowLimit = visualField.bearingToRelativeObstacleDistance.begin()->first + heading; 
    auto highLimit = visualField.bearingToRelativeObstacleDistance.rbegin()->first + heading; 
    avoidOutsideVisualField(lowLimit, highLimit);
    for(auto it : visualField.bearingToRelativeObstacleDistance ){
        bearingAvoidanceSmoothed(it.first + heading, it.second);
        bearingPreferenceSmoothed(it.first + heading, it.second);
   }
}


void ProximityVoter::avoidOutsideVisualField( int16_t visibleFieldLowBearingLimit, 
        int16_t visibleFieldHighBearingLimit)
{
    const auto outsideAvoidanceFactor = 0.5;
    auto vote = courseBallot.maxVotes();
    // less votes for courses where we don't see
    Logger::info("less votes from %d to %d", visibleFieldHighBearingLimit, 
        visibleFieldLowBearingLimit + 360);
    for (auto i = visibleFieldHighBearingLimit; i<visibleFieldLowBearingLimit + 360; ++i)
    {
        courseBallot.add(i, -vote*outsideAvoidanceFactor);
    }
}

//void ProximityVoter::bearingAvoidanceSingleDir(int16_t bearing, int16_t voteAdjust){
//    const auto awayWeight = 1.0;
   // Towards the target, reduce votes
//    courseBallot.add(bearing, voteAdjust);
    // Starboard of the target, increase votes
//    courseBallot.add(bearing + 90, voteAdjust * awayWeight);
//}


///----------------------------------------------------------------------------------
void ProximityVoter::bearingAvoidanceSmoothed( int16_t bearing, uint16_t relativeFreeDistance )
{
    const uint16_t avoidanceBearingRange = 10;
    const double avoidanceNormalization = avoidanceBearingRange;
    const double vote = courseBallot.maxVotes();
    auto normalizedVoteAdjust = 2.0*(100.0 - relativeFreeDistance)/(100.0 * avoidanceNormalization);

    if (relativeFreeDistance < 100){
        Logger::info("Decreasing votes around bearing %d with %f", bearing, vote*normalizedVoteAdjust);
    }
    courseBallot.add(bearing, -vote * normalizedVoteAdjust);
    for(uint16_t j = 1; j < avoidanceBearingRange; j++)
    {
        double voteAdjust = vote * normalizedVoteAdjust * (avoidanceNormalization - j)/avoidanceNormalization;
        courseBallot.add(bearing+j, -voteAdjust);        
        courseBallot.add(bearing-j, -voteAdjust);        
    }
}

void ProximityVoter::bearingPreferenceSmoothed( int16_t bearing, uint16_t relativeFreeDistance )
{
    const uint16_t giveWayAngle = 160;
    const uint16_t preferenceBearingRange = 100;
    const double preferenceNormalization = preferenceBearingRange;
    const double vote = courseBallot.maxVotes();
    auto normalizedVoteAdjust = 2.0*(100.0 - relativeFreeDistance)/(100.0 * preferenceNormalization);

    if (relativeFreeDistance < 100){
        Logger::info("Increasing votes around bearing %d with %f", bearing + giveWayAngle, vote*normalizedVoteAdjust);        
    }
    courseBallot.add(bearing + giveWayAngle, vote * normalizedVoteAdjust);
    for(uint16_t j = 1; j < preferenceBearingRange; j++)
    {
        double voteAdjust =  vote * normalizedVoteAdjust * (preferenceNormalization - j)/preferenceNormalization;
        courseBallot.add(bearing + giveWayAngle + j, voteAdjust);        
        courseBallot.add(bearing + giveWayAngle - j, voteAdjust);        
    }
}

///----------------------------------------------------------------------------------
float ProximityVoter::aisAvoidance( const BoatState_t& boatState, AISCollidable_t& collidable )
{
    const float MIN_DISTANCE = 50.f; // Metres
    const float MAX_DISTANCE = 100.f; // Metres
    const uint16_t AVOIDANCE_BEARING_RANGE = 40;
    uint16_t courseOfEscape = 0;

    float distance = CourseMath::calculateDTW(boatState.lon, boatState.lat, collidable.longitude, collidable.latitude); // in metres
    uint16_t bearing = CourseMath::calculateBTW(boatState.lon, boatState.lat, collidable.longitude, collidable.latitude);

    // Too far away, we don't care
    if( distance < MAX_DISTANCE )
    {

        int16_t bearingDiffStarboard = abs(Utility::headingDifference(bearing, Utility::wrapAngle(collidable.course - 90)));
        int16_t bearingDiffPort = abs(Utility::headingDifference(bearing, Utility::wrapAngle(collidable.course + 90)));

        //Logger::info("Collidable - Bearing: %d Course: %d %d %d", bearing, collidable.course, bearingDiffStarboard, bearingDiffPort);

        // Work out course of escape
        if( bearingDiffStarboard < bearingDiffPort )
        {
            courseOfEscape = Utility::wrapAngle(collidable.course + 90);
            //Logger::info("Course of escape on Starboard of target");
        }
        else
        {
            courseOfEscape = Utility::wrapAngle(collidable.course - 90);
            //Logger::info("Course of escape on port of target");
        }

        for(uint16_t j = 0; j < AVOIDANCE_BEARING_RANGE; j++)
        {
            int16_t vote = (MIN_DISTANCE / distance) * courseBallot.maxVotes();
            // Towards the target's course, reduce votes
            courseBallot.add(courseOfEscape + 180 + j, -(vote / (j + 1)));
            courseBallot.add(courseOfEscape + 180 - j, -(vote / (j + 1)));

            // Away from the target's course, increase votes
            courseBallot.add(courseOfEscape + j, vote - (j/2));
            courseBallot.add(courseOfEscape - j, vote - (j/2));
        }
    }

    return distance;
}
