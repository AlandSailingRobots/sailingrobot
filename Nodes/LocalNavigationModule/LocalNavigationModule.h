/****************************************************************************************
 *
 * File:
 * 		LocalNavigationModule.h
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


#include "../Node.h"
#include "BoatState.h"
#include "ASRVoter.h"
#include "ASRArbiter.h"
#include <vector>


class LocalNavigationModule : Node {
public:
    ///----------------------------------------------------------------------------------
 	/// Constructs the LocalNavigationModule
 	///----------------------------------------------------------------------------------
    LocalNavigationModule( MessageBus& msgBus );

    ///----------------------------------------------------------------------------------
 	/// Does nothing
 	///----------------------------------------------------------------------------------
    bool init();

    ///----------------------------------------------------------------------------------
 	/// Processes the following messages:
    ///     * Vessel State Messages
    ///     * New Waypoint Messages
    ///     * Course Request Message
 	///----------------------------------------------------------------------------------
    void processMessages( const Message* msg );

    ///----------------------------------------------------------------------------------
 	/// Registers a voter, this voter will then be asked to vote when a ballot is held.
 	///----------------------------------------------------------------------------------
    void registerVoter( ASRVoter* voter );

private:
    ///----------------------------------------------------------------------------------
 	/// Starts a ballot, asking every registered voter to vote. Once that is done it
    /// the final result is generated and a desired course message is created.
 	///----------------------------------------------------------------------------------
    void startBallot();

    std::vector<ASRVoter*> voters;
    BoatState_t boatState;
    ASRArbiter arbiter;
};