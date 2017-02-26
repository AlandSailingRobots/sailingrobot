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


#include "ASRCourseBallot.h"


class ASRArbiter {
public:
    ASRArbiter();
    void castVote( ASRCourseBallot& ballot );
    const ASRCourseBallot* getResult() const;

private:
    ASRCourseBallot courseBallot;
};