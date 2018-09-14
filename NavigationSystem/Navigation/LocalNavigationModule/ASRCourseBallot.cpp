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
void ASRCourseBallot::setVeto( uint16_t course )
{
    // Angle wrapping
    course = Utility::wrapAngle( course );

    course = CALCULATE_INDEX( course );
    veto[course] = true;
}

///----------------------------------------------------------------------------------
void ASRCourseBallot::add( uint16_t course, int16_t value )
{
    // Angle wrapping
    course = Utility::wrapAngle( course );

    course = CALCULATE_INDEX( course );
    //value += courses[course];

    /* NOTE: No more cap for the moment.
    // cap the vote
    if( value > MAX_VOTES )
    {
        value = MAX_VOTES;
    }
    */

    courses[course] += value;
}

///----------------------------------------------------------------------------------
void ASRCourseBallot::clear()
{
    memset( courses, 0, sizeof(int16_t) * ASRCourseBallot::ELEMENT_COUNT );
    memset( veto, false, sizeof(bool) * ASRCourseBallot::ELEMENT_COUNT );
    /*std::cout << "Size of courses/veto: " << sizeof(courses) << " " << sizeof(veto) << " "
              << sizeof(*courses) << " " << sizeof(*veto) << " " << sizeof(int16_t) << " " << sizeof(bool) <<std::endl;
    std::cout << "ELEMENT_COUNT * sizeof(int16_t/bool): " << sizeof(int16_t) * ASRCourseBallot::ELEMENT_COUNT << " "
              << sizeof(bool) * ASRCourseBallot::ELEMENT_COUNT << std::endl;*/
    //memset( courses, 0, sizeof(courses) );
    //memset( veto, false, sizeof(veto) );
    
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
bool ASRCourseBallot::getVeto( uint16_t heading ) const
{
     // Angle wrapping
    heading = Utility::wrapAngle( heading );

    heading = CALCULATE_INDEX( heading );

    return veto[heading];
}

///----------------------------------------------------------------------------------
const int16_t* ASRCourseBallot::ptr() const
{
    return courses;
}