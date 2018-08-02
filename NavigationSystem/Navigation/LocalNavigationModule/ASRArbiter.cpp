/****************************************************************************************
 *
 * File:
 * 		ASRArbiter.cpp
 *
 * Purpose:
 *		
 *
 * License:
 *      This file is subject to the terms and conditions defined in the file 
 *      'LICENSE.txt', which is part of this source code package.
 *
 ***************************************************************************************/


#include "ASRArbiter.h"
#include <iostream>


///----------------------------------------------------------------------------------
ASRArbiter::ASRArbiter()
    :courseBallot( ASRArbiter::MAX_VOTES )
{

}

///----------------------------------------------------------------------------------
void ASRArbiter::castVote( const int16_t weight, const ASRCourseBallot& ballot )
{
    //std::lock_guard<std::mutex> lock_guard(m_lock);

    for( uint16_t i = 0; i < 360; i+= ASRCourseBallot::COURSE_RESOLUTION )
    {
        courseBallot.add( i, ballot.get(i) * weight );

        if(ballot.getVeto(i)){ 
            courseBallot.setVeto(i);
        }
    }
}

///----------------------------------------------------------------------------------
void ASRArbiter::castVeto( const ASRCourseBallot& ballot )
{
    for( uint16_t i = 0; i < 360; i+= ASRCourseBallot::COURSE_RESOLUTION )
    {
        if(ballot.getVeto(i)){ 
            courseBallot.setVeto(i);
        }
    }
}

///----------------------------------------------------------------------------------
const uint16_t ASRArbiter::getWinner() const
{
    uint16_t highestIndex;
    int16_t highestValue = -1;

    for( uint16_t i = 0; i < 360; i+= ASRCourseBallot::COURSE_RESOLUTION )
    {
        if( courseBallot.get(i) > highestValue && !courseBallot.getVeto(i) )
        {
            highestIndex = i;
            highestValue = courseBallot.get(i);
        }
    }

    

    printf("Winning Course: %d With Votes: %d\n", highestIndex, highestValue);

    return highestIndex;
}

///----------------------------------------------------------------------------------
const ASRCourseBallot& ASRArbiter::getResult() const
{
    return courseBallot;
}

///----------------------------------------------------------------------------------
 void ASRArbiter::clearBallot()
 {
    courseBallot.clear();
 }