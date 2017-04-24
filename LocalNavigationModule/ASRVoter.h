/****************************************************************************************
 *
 * File:
 * 		ASRVoter.h
 *
 * Purpose:
 *		The abstract base voter class. A voter contains a course ballot and a vote 
 *      function which needs to be overriden. 
 *
 * License:
 *      This file is subject to the terms and conditions defined in the file 
 *      'LICENSE.txt', which is part of this source code package.
 *
 ***************************************************************************************/


#pragma once


#include "ASRCourseBallot.h"
#include "BoatState.h"
#include <string>


class ASRVoter {
public:
    ASRVoter( int16_t maxVotes, float weight, std::string name )
        :courseBallot( maxVotes ), voterWeight( weight ), name(name)
    { }

    ///----------------------------------------------------------------------------------
 	/// Triggers a ASR voter to place votes on the course headings. This function returns
    /// a reference to the internal course ballot data.
 	///----------------------------------------------------------------------------------
    virtual const ASRCourseBallot& vote( const BoatState_t& boatState ) = 0;

    int16_t weight() { return voterWeight; }

    uint16_t getBestCourse(int16_t& value)
    {
        uint16_t bestCourse = 0;
        value = 0;
        for( uint16_t i = 0; i < 360; i++)
        {
            if(courseBallot.get(i) > value)
            {
                value = courseBallot.get(i);
                bestCourse = i;
            }
        }

        return bestCourse;
    }

    std::string getName() { return name; }

protected:
    ASRCourseBallot courseBallot;
    float         voterWeight;
    std::string     name;
};