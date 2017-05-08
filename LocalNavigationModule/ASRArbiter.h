/****************************************************************************************
 *
 * File:
 * 		ASRArbiter.h
 *
 * Purpose:
 *		
 *
 * License:
 *      This file is subject to the terms and conditions defined in the file 
 *      'LICENSE.txt', which is part of this source code package.
 *
 ***************************************************************************************/


#pragma once


#include <stdint.h>
#include "ASRCourseBallot.h"


class ASRArbiter {
public:
    ///----------------------------------------------------------------------------------
 	/// Constructs the Arbiter.
 	///----------------------------------------------------------------------------------
    ASRArbiter();

    ///----------------------------------------------------------------------------------
 	/// Adds all the votes from a course ballot into its internal ballot.
 	///----------------------------------------------------------------------------------
    void castVote( const int16_t weight, const ASRCourseBallot& ballot );

    ///----------------------------------------------------------------------------------
 	/// Returns the winning course.
 	///----------------------------------------------------------------------------------
    const uint16_t getWinner() const;

    ///----------------------------------------------------------------------------------
 	/// Returns the summed results of all the voters that have cast their vote.
 	///----------------------------------------------------------------------------------
    const ASRCourseBallot& getResult() const;

    ///----------------------------------------------------------------------------------
 	/// Clears the current ballot
 	///----------------------------------------------------------------------------------
    void clearBallot();
private:
    const int MAX_VOTES = 25;
    ASRCourseBallot courseBallot;
};