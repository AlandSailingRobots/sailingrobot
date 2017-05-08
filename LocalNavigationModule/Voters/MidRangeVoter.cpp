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

    static const double MIN_DISTANCE = 100;//100; // 200 Metres
    static const double MAX_DISTANCE = 1000; // 1KM
    static const double SAFE_DISTANCE = 100;

    CollidableList<AISCollidable_t> aisContacts = collidableMgr.getAISContacts();

    for(uint16_t i = 0; i < 360; i++)
    {
        float closestCPA = 10000;
        //float closestTime = 0;
        double riskOfCollision = 0;
        float distance = 0;

        for(uint16_t j = 0; j < aisContacts.length(); j++)
        {
            AISCollidable_t collidable = aisContacts.next();
            distance = CourseMath::calculateDTW(boatState.lon, boatState.lat, collidable.longitude, collidable.latitude);

            if(distance < MIN_DISTANCE || distance > MAX_DISTANCE)
            {
                continue;
            }

            double time = 0;
            float cpa = getCPA( collidable, boatState, i, time);

            if(cpa < closestCPA && cpa < SAFE_DISTANCE && cpa >= 0)
            {
                closestCPA = cpa;
            }
        }

        if(closestCPA < SAFE_DISTANCE)
        {
            riskOfCollision = (SAFE_DISTANCE - closestCPA) / SAFE_DISTANCE;
        }
        assignVotes(i, riskOfCollision);

        if(i == 223)
        {
            Logger::info("CPA at tack: %f so votes is %d", closestCPA, courseBallot.get(i));
            Logger::info("Distance: %f ", distance);
        }

        aisContacts.reset();
    }

    return courseBallot;
}

///----------------------------------------------------------------------------------
const void MidRangeVoter::assignVotes( uint16_t course, float collisionRisk )
{
    int16_t vote = courseBallot.maxVotes() - (collisionRisk * courseBallot.maxVotes());
    courseBallot.add(course, vote);
}

///----------------------------------------------------------------------------------
void calculateVelocity( uint16_t course, double speed, double& x, double& y)
{
    double courseR = course * (M_PI / 180);
    x = cos(courseR) * speed;
    y = sin(courseR) * speed;
}

///----------------------------------------------------------------------------------
/// Based on work from http://www.ai.sri.com/geovrml/rhumbline/html/sld003.htm
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
        return -1;
    }

    double squaredVelocity = dVX * dVX + dVY * dVY;
    double dotProduct = 2 * coursesDotProduct;

    double cpa = sqrt(distance * distance - ( (dotProduct * dotProduct) / (4 * squaredVelocity) ));
    
    time = ( -dotProduct / (2 * squaredVelocity) );

    if(time < 0)
    {
        time = 0;
        return -1;
    }

    return cpa;
}