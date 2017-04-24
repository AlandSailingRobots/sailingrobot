/****************************************************************************************
 *
 * File:
 * 		CollidableMgr.h
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

    void addAISContact(uint32_t mmsi, float lat, float lon, float speed, uint16_t course);
    void addVisualContact(uint32_t id, uint16_t bearing);

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