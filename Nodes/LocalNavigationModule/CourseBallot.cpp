 /****************************************************************************************
 *
 * File:
 * 		.cpp
 *
 * Purpose:
 *		
 *
 * License:
 *      This file is subject to the terms and conditions defined in the file 
 *      'LICENSE.txt', which is part of this source code package.
 *
 ***************************************************************************************/


#include "CourseBallot.h"


///----------------------------------------------------------------------------------
CourseBallot::CourseBallot( int16_t maxVotes )
    :MAX_VOTES( maxVotes )
{
    clear();
}

///----------------------------------------------------------------------------------
void CourseBallot::set( uint16_t course, int16_t value )
{
    // Angle wrapping
    course = wrapAngle( course );

    // cap the vote
    if( value > MAX_VOTES )
    {
        value = MAX_VOTES;
    }

    // ensure the index exists based on the course resolution selected
    if( course % CourseBallot::COURSE_RESOLUTION == 0 )
    {
        courses[course] = value;
    }
}

///----------------------------------------------------------------------------------
void CourseBallot::add( uint16_t course, int16_t value )
{
    // Angle wrapping
    course = wrapAngle( course );

    // ensure the index exists based on the course resolution selected
    if( course % CourseBallot::COURSE_RESOLUTION == 0 )
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
void CourseBallot::clear()
{
    memset( courses, 0, sizeof(int16_t) * CourseBallot::ELEMENT_COUNT );
}

///----------------------------------------------------------------------------------
const int16_t* CourseBallot::ptr()
{
    return courses;
}

///----------------------------------------------------------------------------------
int16_t CourseBallot::wrapAngle( int16_t angle )
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