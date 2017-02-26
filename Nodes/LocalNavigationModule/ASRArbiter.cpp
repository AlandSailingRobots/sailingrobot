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


///----------------------------------------------------------------------------------
ASRArbiter::ASRArbiter()
    :courseBallot( ASRArbiter::MAX_VOTES )
{

}

///----------------------------------------------------------------------------------
void ASRArbiter::castVote( const ASRCourseBallot& ballot )
{
    for( uint16_t i = 0; i < 360; i+= ASRCourseBallot::COURSE_RESOLUTION )
    {
        courseBallot.add( i, ballot.get(i) );
    }
}

///----------------------------------------------------------------------------------
const uint16_t ASRArbiter::getWinner() const
{
    uint16_t highestIndex;
    int16_t highestValue = -1;

    for( uint16_t i = 0; i < 360; i+= ASRCourseBallot::COURSE_RESOLUTION )
    {
        if( courseBallot.get(i) > highestValue )
        {
            highestIndex = i;
            highestValue = courseBallot.get(i);
        }
    }

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