/****************************************************************************************
 *
 * File:
 * 		CollidableMgr.h
 *
 * Purpose:
 *    Handles the objects we can collide with, vessels found by the AIS
 *    or smaller boats/obstacles found by the thermal imager
 *    The AISProcessing adds/updates the data to the collidableMgr
 *    Removes the data when enough time has gone without the contact being updated
 * License:
 *      This file is subject to the terms and conditions defined in the file
 *      'LICENSE.txt', which is part of this source code package.
 *
 ***************************************************************************************/


#pragma once


#include "Collidable.h"
#include "CollidableList.h"
#include <mutex>
#include <stdint.h>
#include <thread>


class CollidableMgr {
public:
    CollidableMgr();

    ///----------------------------------------------------------------------------------
    /// Starts the garbage collector that cleans up old contacts.
    ///----------------------------------------------------------------------------------
    void startGC();

    void addAISContact(uint32_t mmsi, double lat, double lon, float speed, float course);
    void addAISContact(uint32_t mmsi, float length, float beam);
    void addVisualContact( uint16_t n, uint16_t bearing );

    CollidableList<AISCollidable_t> getAISContacts();
    CollidableList<VisualCollidable_t> getVisualContacts();

    void removeOldContacts();
private:
    static void ContactGC(CollidableMgr* ptr);

    std::vector<AISCollidable_t> aisContacts;
    std::vector<VisualCollidable_t> visualContacts;
    std::mutex aisListMutex;
    std::mutex visualListMutex;
    bool ownAISLock;
    bool ownVisualLock;
    std::thread* m_Thread;
};
