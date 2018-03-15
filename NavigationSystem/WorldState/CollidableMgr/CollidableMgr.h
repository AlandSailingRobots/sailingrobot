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
    // replaces the visual field
    void addVisualField(std::map<int16_t, uint16_t> relBearingToRelObstacleDistance, int16_t heading);

    CollidableList<AISCollidable_t> getAISContacts();
    VisualField_t getVisualField();

    void removeOldVisualField();

    void removeOldAISContacts();
private:
    static void ContactGC(CollidableMgr* ptr);

    std::vector<AISCollidable_t> aisContacts;
    VisualField_t m_visualField;
    std::mutex aisListMutex;
    std::mutex m_visualMutex;
    bool ownAISLock;
    std::thread* m_Thread;
};
