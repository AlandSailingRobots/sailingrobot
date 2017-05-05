/****************************************************************************************
 *
 * File:
 * 		WindVoter.h
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


#include "../ASRVoter.h"

#include <vector>


class WindVoter : public ASRVoter {
public:
    ///----------------------------------------------------------------------------------
 	/// Constructs the wind voter.
 	///----------------------------------------------------------------------------------
    WindVoter( int16_t maxVotes, int16_t weight );

    ///----------------------------------------------------------------------------------
 	/// Triggers a ASR voter to place votes on the course headings. The wind voter
    /// places voters on courses that allow the boat to go faster
 	///----------------------------------------------------------------------------------
    const ASRCourseBallot& vote( const BoatState_t& boatState );
private:
    const int TW_BUFFER_SIZE = 2;
    std::vector<float> trueWindBuffer;
};