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
    course = wrapAngle( course );

    // cap the vote
    if( value > MAX_VOTES )
    {
        value = MAX_VOTES;
    }

    // ensure the index exists based on the course resolution selected
    if( course % ASRCourseBallot::COURSE_RESOLUTION == 0 )
    {
        courses[course] = value;
    }
}

///----------------------------------------------------------------------------------
void ASRCourseBallot::add( uint16_t course, int16_t value )
{
    // Angle wrapping
    course = wrapAngle( course );

    // ensure the index exists based on the course resolution selected
    if( course % ASRCourseBallot::COURSE_RESOLUTION == 0 )
    {
        value += courses[course];

        // cap the vote
        if( value > MAX_VOTES )
        {
            value = MAX_VOTES;
        }

        courses[course] = value;
    }
}

///----------------------------------------------------------------------------------
void ASRCourseBallot::clear()
{
    memset( courses, 0, sizeof(int16_t) * ASRCourseBallot::ELEMENT_COUNT );
}

///----------------------------------------------------------------------------------
const int16_t* ASRCourseBallot::ptr() const
{
    return courses;
}

///----------------------------------------------------------------------------------
int16_t ASRCourseBallot::wrapAngle( int16_t angle ) const
{
    while ( angle < 0 || angle > 360 )
    {
        if ( angle < 0 )
        {
            angle += 360;
        }
        else
        {
            angle -= 360;
        }
    }

    return angle;
}