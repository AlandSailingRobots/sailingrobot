/****************************************************************************************
 *
 * File:
 * 		CourseVoter.cpp
 *
 * Purpose:
 *		
 *
 * License:
 *      This file is subject to the terms and conditions defined in the file 
 *      'LICENSE.txt', which is part of this source code package.
 *
 ***************************************************************************************/


#include "CourseVoter.h"
#include "../ASRVoter.h"
#include "../Math/CourseMath.h"
#include "../SystemServices/Logger.h"

#include <iostream>


///----------------------------------------------------------------------------------
CourseVoter::CourseVoter( int16_t maxVotes, int16_t weight )
    :ASRVoter( maxVotes, weight, "Course" )
{

}

///----------------------------------------------------------------------------------
const ASRCourseBallot& CourseVoter::vote( const BoatState_t& boatState )
{
    courseBallot.clear();
    
    // Add votes to the direction the boat is facing, less cost to change the vessel.
    for( int i = 0; i < 10; i += ASRCourseBallot::COURSE_RESOLUTION )
    {
        courseBallot.add( boatState.heading + i, (( 10 - i ) / 10) * (courseBallot.maxVotes() / 10) );
        courseBallot.add( boatState.heading - i, (( 10 - i ) / 10) * (courseBallot.maxVotes() / 10) );
    }

    return courseBallot;
} 
