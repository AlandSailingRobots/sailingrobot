 /****************************************************************************************
 *
 * File:
 * 		ASRCourseBallot.cpp
 *
 * Purpose:
 *		
 *
 * License:
 *      This file is subject to the terms and conditions defined in the file 
 *      'LICENSE.txt', which is part of this source code package.
 *
 ***************************************************************************************/


#include "ASRCourseBallot.h"
#include "Math/Utility.h"


#define CALCULATE_INDEX( index ) index / ASRCourseBallot::COURSE_RESOLUTION;


///----------------------------------------------------------------------------------
ASRCourseBallot::ASRCourseBallot( int16_t maxVotes )
    :MAX_VOTES( maxVotes )
{
    clear();
}

///----------------------------------------------------------------------------------
void ASRCourseBallot::set( uint16_t course, int16_t value )
{
    // Angle wrapping
    course = Utility::wrapAngle( course );

    // cap the vote
    if( value > MAX_VOTES )
    {
        value = MAX_VOTES;
    }

    course = CALCULATE_INDEX( course );
    courses[course] = value;
}

///----------------------------------------------------------------------------------
void ASRCourseBallot::add( uint16_t course, int16_t value )
{
    // Angle wrapping
    course = Utility::wrapAngle( course );

    course = CALCULATE_INDEX( course );
    value += courses[course];

    // cap the vote
    if( value > MAX_VOTES )
    {
        value = MAX_VOTES;
    }

    if( value < 0)
    {
        value = 0;
    }

    courses[course] = value;
}

///----------------------------------------------------------------------------------
void ASRCourseBallot::clear()
{
    memset( courses, 0, sizeof(int16_t) * ASRCourseBallot::ELEMENT_COUNT );
}

///----------------------------------------------------------------------------------
int16_t ASRCourseBallot::get( uint16_t heading ) const
{
     // Angle wrapping
    heading = Utility::wrapAngle( heading );

    heading = CALCULATE_INDEX( heading );

    return courses[heading];
}

///----------------------------------------------------------------------------------
const int16_t* ASRCourseBallot::ptr() const
{
    return courses;
}