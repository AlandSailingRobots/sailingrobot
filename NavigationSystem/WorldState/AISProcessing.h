/****************************************************************************************
 *
 * File:
 * 		AISProcessing.h
 *
 * Purpose:
 *     Receives the data from the CANAISNode and processes it and sends the vessels
 *     that are in a certain radius to the collidableMgr
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/

#pragma once

#include "../Database/DBHandler.h"
#include "../Math/CourseMath.h"
#include "../MessageBus/ActiveNode.h"
#include "../MessageBus/Message.h"
#include "../Messages/AISDataMsg.h"
#include "../Messages/StateMessage.h"
#include "../SystemServices/Logger.h"
#include "../SystemServices/Timer.h"
#include "../WorldState/CollidableMgr/CollidableMgr.h"

#include <chrono>
#include <mutex>
#include <thread>
#include <vector>

class AISProcessing : public ActiveNode {
   public:
    /*
     * Constructor, pointer to messagebus and canservice
     * int radius, the distance [meter] from us in which a vessel is interesting
     * uint32_t mmsi, the ID number of our shown vessel, makes sure it gets ignored and not added to
     * the collidable manager double loopTime, how often we send messages
     */
    AISProcessing(MessageBus& msgBus, DBHandler& dbhandler, CollidableMgr* collidableMgr);
    ~AISProcessing();

    bool init();

    /*
     * Processes the message received
     */
    void processMessage(const Message* msg);

    /*
     * Starts the worker thread
     */
    void start();

    /*
     * Stops the worker thread
     */
    void stop();

   private:
    /*
     * Update values from the database as the loop time of the thread and others parameters
     */
    void updateConfigsFromDB();

    /*
     * Sends the data to the collidable manager
     */
    void addAISDataToCollidableMgr();

    /*
     * Gets to process the message if the message received is an AISDataMsg
     */
    void processAISMessage(AISDataMsg* msg);

    /*
     * The function that thread works on
     */
    static void AISProcessingThreadFunc(ActiveNode* nodePtr);

    /*
     * Private variables
     */
    std::vector<AISVessel> m_Vessels;
    std::vector<AISVesselInfo> m_InfoList;
    double m_latitude;
    double m_longitude;
    double m_LoopTime;
    int m_Radius;
    uint32_t m_MMSI;
    std::mutex m_lock;
    CollidableMgr* collidableMgr;
    DBHandler& m_db;
    std::atomic<bool> m_running;
};
