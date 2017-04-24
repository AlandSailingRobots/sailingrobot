/****************************************************************************************
 *
 * File:
 * 		MidRangeVoter.h
 *
 * Purpose:
 *		
 *
 * License:
 *      This file is subject to the terms and conditions defined in the file 
 *      'LICENSE.txt', which is part of this source code package.
 *
 ***************************************************************************************/


#include "MidRangeVoter.h"


#include "Math/CourseMath.h"
#include "Math/Utility.h"
#include "SystemServices/Logger.h"
#include <cmath>


///----------------------------------------------------------------------------------
MidRangeVoter::MidRangeVoter( int16_t maxVotes, int16_t weight, CollidableMgr& collisionMgr )
    :ASRVoter( maxVotes, weight, "MidRange Voter" ), collidableMgr(collisionMgr)
{

}

///----------------------------------------------------------------------------------
const ASRCourseBallot& MidRangeVoter::vote( const BoatState_t& boatState )
{
    courseBallot.clear();

    static const double MIN_DISTANCE = 100; // 200 Metres
    static const double MAX_DISTANCE = 1000; // 1KM

    CollidableList<AISCollidable_t> aisContacts = collidableMgr.getAISContacts();

    // AIS Contacts
    for(uint16_t i = 0; i < aisContacts.length(); i++)
    {
        AISCollidable_t collidable = aisContacts.next();
        float distance = CourseMath::calculateDTW(boatState.lon, boatState.lat, collidable.longitude, collidable.latitude);
        Logger::info("Distance: %f ", distance);
        if( distance > MIN_DISTANCE && distance < MAX_DISTANCE )
        {
            float closestCPA = 10000;
            float closestTime = 0;
            int16_t closestCourse = 0;

            for( uint16_t i = 0; i < 360; i++)
            {
                double time = 0;
                float cpa = getCPA( collidable, boatState, i, time);

                if(cpa < closestCPA && cpa > 0)
                {
                    closestCPA = cpa;
                    closestCourse = i;
                    closestTime = time;
                }

                if(cpa > MIN_DISTANCE && cpa < MAX_DISTANCE)
                {
                    uint16_t perpCourse = perpendicularCourse( collidable, boatState );
                    double riskOfCollision = 1 - ((cpa - MIN_DISTANCE) / (MAX_DISTANCE - MIN_DISTANCE));
                    //Logger::info("Risk Of collision: %f CPA: %f", riskOfCollision, cpa);
                    assignVotes(perpCourse, riskOfCollision);
                }
            }

            Logger::info("Vessel: %d Course: %d CPA: %f Time: %f minutes", collidable.mmsi, closestCourse, closestCPA, closestTime / 60);
        }
    }

    return courseBallot;
}

///----------------------------------------------------------------------------------
const void MidRangeVoter::assignVotes( uint16_t perpCourse, float collisionRisk )
{
    uint16_t AVOIDANCE_BEARING_RANGE = 30;

    for(uint16_t j = 0; j < AVOIDANCE_BEARING_RANGE; j++)
    {
        int16_t vote = collisionRisk * courseBallot.maxVotes();
        // Towards the target's course, reduce votes
        courseBallot.add(perpCourse + 180 + j, -(vote / (j + 1)));
        courseBallot.add(perpCourse + 180 - j, -(vote / (j + 1)));

        // Away from the target's course, increase votes
        courseBallot.add(perpCourse + j, vote - (j/2));
        courseBallot.add(perpCourse - j, vote - (j/2));
    } 
}

///----------------------------------------------------------------------------------
void calculateVelocity( uint16_t course, double speed, double& x, double& y)
{
    double courseR = course * (M_PI / 180);
    x = cos(courseR) * speed;
    y = sin(courseR) * speed;
}

///----------------------------------------------------------------------------------
const double MidRangeVoter::getCPA( const AISCollidable_t& collidable, const BoatState_t& boatState, uint16_t course, double& time)
{
    double DEG_TO_RAD = M_PI / 180;

    float distance = CourseMath::calculateDTW(boatState.lon, boatState.lat, collidable.longitude, collidable.latitude); // in metres
    uint16_t bearing = CourseMath::calculateBTW(boatState.lon, boatState.lat, collidable.longitude, collidable.latitude);

    // Work out x and y coordinate relative to ASV
    double xRel = distance * cos( DEG_TO_RAD * bearing );
    double yRel = distance * sin( DEG_TO_RAD * bearing );

    // Work out velocity 
    double asv_vX = 0;
    double asv_vY = 0;
    double vessel_vX = 0;
    double vessel_vY = 0;

    calculateVelocity( course, boatState.speed, asv_vX, asv_vY );
    calculateVelocity( collidable.course, collidable.speed, vessel_vX, vessel_vY );

    double dVX = vessel_vX - asv_vX;
    double dVY = vessel_vY - asv_vY; 

    // Work out if our course is parallel
    double coursesDotProduct = (xRel * dVX) + (yRel * dVY);  
    if (coursesDotProduct == 0.0) 
    {
        return 0;
    }

    double a = dVX * dVX + dVY * dVY;
    double b = 2 * coursesDotProduct;

    double cpa = distance * distance - ((b*b)/(4*a));
    
    time = (-b/(2*a));

    if(time < 0)
    {
        time = 0;
        return 0;
    }

    return sqrt(cpa);
}

///----------------------------------------------------------------------------------
const uint16_t MidRangeVoter::perpendicularCourse( const AISCollidable_t& collidable, const BoatState_t& boatState)
{
    uint16_t perpCourse = 0;

    uint16_t bearing = CourseMath::calculateBTW(boatState.lon, boatState.lat, collidable.longitude, collidable.latitude);

    int16_t bearingDiffStarboard = abs(Utility::headingDifference(bearing, Utility::wrapAngle(collidable.course - 90)));
    int16_t bearingDiffPort = abs(Utility::headingDifference(bearing, Utility::wrapAngle(collidable.course + 90)));

    if( bearingDiffStarboard < bearingDiffPort )
    {
        perpCourse = Utility::wrapAngle(collidable.course + 90);
    }
    else
    {
        perpCourse = Utility::wrapAngle(collidable.course - 90);
    }

    return perpCourse;
}