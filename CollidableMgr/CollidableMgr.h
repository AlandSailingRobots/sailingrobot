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


class CollidableMgr {
public:
    CollidableMgr();
    void addAISContact(uint32_t mmsi, float lat, float lon, float speed, uint16_t course);
    void addVisualContact(uint32_t id, uint16_t bearing);

    CollidableList<AISCollidable_t> getAISContacts();
    CollidableList<VisualCollidable_t> getVisualContacts();
private:
    std::vector<AISCollidable_t> aisContacts;
    std::vector<VisualCollidable_t> visualContacts;
    std::mutex aisListMutex;
    std::mutex visualListMutex;
    bool ownAISLock;
    bool ownVisualLock;
};