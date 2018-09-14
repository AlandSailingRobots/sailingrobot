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
#include "Navigation/LocalNavigationModule/ASRVoter.h"
#include "Math/CourseMath.h"
#include "SystemServices/Logger.h"

#include <iostream>


///----------------------------------------------------------------------------------
CourseVoter::CourseVoter( int16_t maxVotes, int16_t weight )
    :ASRVoter( maxVotes, weight, "Course" )
{

}

///----------------------------------------------------------------------------------
const ASRCourseBallot& CourseVoter::vote( const BoatState_t& boatState )
{
    //std::lock_guard<std::mutex> lock_guard(m_lock);

    courseBallot.clear();
    
    // Add votes to the direction the boat is facing, less cost to change the vessel.
    for( int i = 0; i < 10; i += ASRCourseBallot::COURSE_RESOLUTION )
    {
        courseBallot.add( boatState.heading + i, (( 10.0 - i ) / 10.0) * courseBallot.maxVotes() );
        courseBallot.add( boatState.heading - i, (( 10.0 - i ) / 10.0) * courseBallot.maxVotes() );

        //std::cout << "Looping: " << i << std::endl;
    }
    //Negate doubled value on i=0
    courseBallot.add( boatState.heading, -(courseBallot.maxVotes() / 10.0) );

    //int num_non_zero = std::count_if( std::begin(courseBallot.courses), std::end(courseBallot.courses), [](int16_t i){return i>0;} );
    //std::cout << "Number of non zero: " << num_non_zero << std::endl;

    return courseBallot;
} 
