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


class ASRVoter {
public:
    ASRVoter( int16_t maxVotes, int16_t weight )
        :courseBallot( maxVotes ), voterWeight( weight )
    { }

    ///----------------------------------------------------------------------------------
 	/// Triggers a ASR voter to place votes on the course headings. This function returns
    /// a reference to the internal course ballot data.
 	///----------------------------------------------------------------------------------
    virtual const ASRCourseBallot& vote( const BoatState_t& boatState ) = 0;

    int16_t weight() { return voterWeight; }

protected:
    ASRCourseBallot courseBallot;
    int16_t         voterWeight;
};