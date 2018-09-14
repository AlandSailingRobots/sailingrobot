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

 #include <algorithm>


///----------------------------------------------------------------------------------
MidRangeVoter::MidRangeVoter( int16_t maxVotes, int16_t weight, CollidableMgr& collisionMgr )
    :ASRVoter( maxVotes, weight, "MidRange Voter" ), collidableMgr(collisionMgr)
{

}

///----------------------------------------------------------------------------------
const ASRCourseBallot& MidRangeVoter::vote( const BoatState_t& boatState )
{
    /*
    * Changing so that safe distance takes vessel size into account
    * Previously: We had a constant SAFE_DISTANCE = 100 and didn't account for size at all
    * New, improved and groundbreaking way:
    * A default safe distance that we use if size data is unavailable or it is a smaller vessel
    * Otherwise the safe distance is 1.5 times it's length, meaning we want to stay 300 meters clear
    * of a vessel that is 200 meters long
    * cpa_weight is to make sure that if we are close to multiple vessels, the larger vessel
    * will be prioritized even though we may have a smaller cpa for the smaller vessel
    */
    courseBallot.clear();

    static const double MIN_DISTANCE = 100;//100; // 200 Metres
    static const double MAX_DISTANCE = 1000; // 1KM
    static const double DEFAULT_SAFE_DISTANCE = 100;
    double SAFE_DISTANCE, cpa_weight, cpa_current_weight = 1., safe_dist_cpa = DEFAULT_SAFE_DISTANCE;
    CollidableList<AISCollidable_t> aisContacts = collidableMgr.getAISContacts();

    //std::cout << "Mid range max vote (before loop): " << *std::max_element(std::begin(courseBallot.courses),std::end(courseBallot.courses)) << std::endl;
    for(uint16_t i = 0; i < 360; i++) // NOTE: not using ASRCourseBallot::COURSE_RESOLUTION
    {
        float closestCPA = 10000;
        //float closestTime = 0;
        double riskOfCollision = 0;
        float distance = 0;
        SAFE_DISTANCE = DEFAULT_SAFE_DISTANCE;

        for(uint16_t j = 0; j < aisContacts.length(); j++)
        {
            //std::cout << "In AIS Contacts loop" << std::endl;
            AISCollidable_t collidable = aisContacts.next();
            distance = CourseMath::calculateDTW(boatState.lon, boatState.lat, collidable.longitude, collidable.latitude);
            //std::cout << "Boat lon/lat and Collidable lon/lat: " << boatState.lon << " / " << boatState.lat << std::endl;
            //std::cout << "                                     " << collidable.longitude << " / " << collidable.latitude << std::endl;
            //std::cout << "AIS Distance value: " << distance << std::endl;
//            std::cout << "Parameters for distance computation: " << std::endl;
//            std::cout << "boatState lon/lat: " << boatState.lon << " " << boatState.lat << std::endl;
//            std::cout << "collidable lon/lat: " << collidable.longitude << " " << collidable.latitude << std::endl;

            if(distance < MIN_DISTANCE || distance > MAX_DISTANCE)
            {
//                std::cout << "Nothing to do for this distance" << std::endl;
                continue;
            }

            if (collidable.length != 0 && collidable.beam != 0) { //Make sure size data is available

              SAFE_DISTANCE = std::max(SAFE_DISTANCE, 1.5*collidable.length);
            }

            double time = 0;
            float cpa = getCPA( collidable, boatState, i, time);
            cpa_weight = DEFAULT_SAFE_DISTANCE/SAFE_DISTANCE;
            if(std::max(20.0,cpa*cpa_weight) < closestCPA*cpa_current_weight && cpa < SAFE_DISTANCE && cpa >= 0)
            {
                closestCPA = cpa;
                safe_dist_cpa = SAFE_DISTANCE;
                cpa_current_weight = cpa_weight;
            }
        }

        if(closestCPA < safe_dist_cpa)
        {
            //std::cout << "In closestCPA conditional" << std::endl;
            riskOfCollision = (safe_dist_cpa - closestCPA) / safe_dist_cpa;
        }
        //std::cout << "Risk value: " << riskOfCollision << std::endl;
        assignVotes(i, riskOfCollision);
        //std::cout << "Mid range min vote (after assignement " << i << "): " << *std::min_element(std::begin(courseBallot.courses),std::end(courseBallot.courses)) << std::endl;

        if(closestCPA < 100)
        {
        Logger::debug("CPA %f at %d so votes is %d", closestCPA, i, courseBallot.get(i));
        Logger::debug("Distance: %f ", distance);
        }

        aisContacts.reset();
    }
    //std::cout << "Mid range min vote: " << *std::min_element(std::begin(courseBallot.courses),std::end(courseBallot.courses)) << std::endl;
    //std::min_element(std::begin(playerSums), std::end(playerSums));
    //std::cout << "Max vote default param = " << courseBallot.maxVotes() << std::endl;

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
    //std::cout << "CPA Value: " << cpa << std::endl;

    return cpa;
}
