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


#include "MessageBus/ActiveNode.h"
#include "DataBase/DBHandler.h"
#include "BoatState.h"
#include "ASRVoter.h"
#include "ASRArbiter.h"
#include <vector>


class LocalNavigationModule : public ActiveNode {
public:
    ///----------------------------------------------------------------------------------
 	/// Constructs the LocalNavigationModule
 	///----------------------------------------------------------------------------------
    LocalNavigationModule( MessageBus& msgBus, DBHandler& dbhandler);

    ///----------------------------------------------------------------------------------
 	/// Does nothing
 	///----------------------------------------------------------------------------------
    bool init();

    ///----------------------------------------------------------------------------------
 	/// Starts the quick hack wakeup thread
 	///----------------------------------------------------------------------------------
    void start();

    ///----------------------------------------------------------------------------------
 	/// Processes the following messages:
    ///     * GPS Data Messages
    ///     * Compass Messages
    ///     * Wind Messages
    ///     * New Waypoint Messages
    ///     * Course Request Message
 	///----------------------------------------------------------------------------------
    void processMessage( const Message* msg );

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

    ///----------------------------------------------------------------------------------
	/// Update values from the database as the loop time of the thread and others parameters
	///----------------------------------------------------------------------------------
    void updateConfigsFromDB();

    ///----------------------------------------------------------------------------------
 	/// Just a little hack for waking up the navigation module for now
 	///----------------------------------------------------------------------------------
    static void WakeupThreadFunc( ActiveNode* nodePtr );

    std::vector<ASRVoter*> voters;
    BoatState_t boatState;
    ASRArbiter arbiter;
    double m_LoopTime;
    DBHandler& m_db;
};
