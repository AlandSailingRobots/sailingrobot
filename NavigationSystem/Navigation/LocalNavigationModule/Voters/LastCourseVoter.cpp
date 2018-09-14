/****************************************************************************************
 *
 * File:
 * 		LastCourseVoter.cpp
 *
 * Purpose:
 *		Vote for the last winning course
 *
 * License:
 *      This file is subject to the terms and conditions defined in the file
 *      'LICENSE.txt', which is part of this source code package.
 *
 ***************************************************************************************/


#include "LastCourseVoter.h"
#include "Navigation/LocalNavigationModule/ASRVoter.h"
#include "Math/CourseMath.h"
#include "SystemServices/Logger.h"

#include <iostream>


///----------------------------------------------------------------------------------
LastCourseVoter::LastCourseVoter( int16_t maxVotes, int16_t weight )
        :ASRVoter( maxVotes, weight, "Course" )
{

}

///----------------------------------------------------------------------------------
const ASRCourseBallot& LastCourseVoter::vote( const BoatState_t& boatState )
{

    courseBallot.clear();

    // Add votes to the last direction the boat has chosen.
    for( int i = 0; i < 10; i += ASRCourseBallot::COURSE_RESOLUTION )
    {
        courseBallot.add( lastWinningBearing + i, (( 10.0 - i ) / 10.0) * courseBallot.maxVotes() );
        courseBallot.add( lastWinningBearing - i, (( 10.0 - i ) / 10.0) * courseBallot.maxVotes() );
    }
    //Negate doubled value on i=0
    courseBallot.add( lastWinningBearing, -courseBallot.maxVotes() );


    return courseBallot;
}
