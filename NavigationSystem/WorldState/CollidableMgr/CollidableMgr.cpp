/****************************************************************************************
 *
 * File:
 * 		CollidableMgr.cpp
 *
 * Purpose:
 *    Handles the objects we can collide with, vessels found by the AIS
 *    or smaller boats/obstacles found by the thermal imager
 *    The AISProcessing adds/updates the data to the collidableMgr
 *    Removes the data when enough time has gone without the contact being updated
 *
 * Developer notes:
 *  NOTE: Change how we remove AIS contacts,
 *        rather than removing after a certain time,
 *        remove when the contact is outside the radius of interest
 *  NOTE: Use a smaller, separate loop time in ContactGC
 *
 * License:
 *      This file is subject to the terms and conditions defined in the file
 *      'LICENSE.txt', which is part of this source code package.
 *
 ***************************************************************************************/

#include "CollidableMgr.h"
#include "SystemServices/SysClock.h"
#include "SystemServices/Logger.h"
#include <chrono>


#define AIS_CONTACT_TIME_OUT        600        // 10 Minutes
#define VISUAL_CONTACT_TIME_OUT 120
#define NOT_AVAILABLE           -2000

///----------------------------------------------------------------------------------
CollidableMgr::CollidableMgr()
    :ownAISLock(false), ownVisualLock(false)
{

}

///----------------------------------------------------------------------------------
void CollidableMgr::startGC()
{
    m_Thread = new std::thread(ContactGC, this);
}

///----------------------------------------------------------------------------------
void CollidableMgr::addAISContact( uint32_t mmsi, double lat, double lon, float speed, float course )
{
    if( !this->ownAISLock )
    {
        this->aisListMutex.lock();
        this->ownAISLock = true;
    }

    // Check if the contact already exists, and if so update it
    bool contactExists = false;
    for( uint16_t i = 0; i < this->aisContacts.size() && !contactExists; i++ )
    {
        if( this->aisContacts[i].mmsi == mmsi)
        {
            this->aisContacts[i].latitude = lat;
            this->aisContacts[i].longitude = lon;
            this->aisContacts[i].speed = speed;
            this->aisContacts[i].course = course;
            this->aisContacts[i].lastUpdated = SysClock::unixTime();
            contactExists = true;
        }
    }

    if(!contactExists)
    {
        AISCollidable_t aisContact;
        aisContact.mmsi = mmsi;
        aisContact.latitude = lat;
        aisContact.longitude = lon;
        aisContact.speed = speed;
        aisContact.course = course;
        aisContact.length = NOT_AVAILABLE;
        aisContact.beam = NOT_AVAILABLE;
        aisContact.lastUpdated = SysClock::unixTime();

        this->aisContacts.push_back(aisContact);
      }
    this->aisListMutex.unlock();
    this->ownAISLock = false;
}

///----------------------------------------------------------------------------------
void CollidableMgr::addAISContact( uint32_t mmsi, float length, float beam )
{
    if( !this->ownAISLock )
    {
        this->aisListMutex.lock();
        this->ownAISLock = true;
    }

    // Check if the contact already exists, and if so update it
    bool contactExists = false;
    for( uint16_t i = 0; i < this->aisContacts.size() && !contactExists; i++ )
    {
        if( this->aisContacts[i].mmsi == mmsi)
        {
            this->aisContacts[i].length = length;
            this->aisContacts[i].beam = beam;
            contactExists = true;
        }
    }

    if(!contactExists)
    {
        AISCollidable_t aisContact;
        aisContact.mmsi = mmsi;
        aisContact.length = length;
        aisContact.beam = beam;
        aisContact.latitude = NOT_AVAILABLE;
        aisContact.longitude = NOT_AVAILABLE;
        aisContact.speed = NOT_AVAILABLE;
        aisContact.course = NOT_AVAILABLE;
        aisContact.lastUpdated = SysClock::unixTime();

        this->aisContacts.push_back(aisContact);
      }
    this->aisListMutex.unlock();
    this->ownAISLock = false;
}

///----------------------------------------------------------------------------------
void CollidableMgr::addVisualContact( uint32_t id, uint16_t bearing )
{
    if( !this->ownVisualLock )
    {
        this->visualListMutex.lock();
        this->ownVisualLock = true;
    }

   // Check if the contact already exists, and if so update it
    bool contactExists = false;
    for( uint16_t i = 0; i < this->visualContacts.size() && !contactExists; i++ )
    {
        if( this->visualContacts[i].id == id)
        {
            this->visualContacts[i].bearing = bearing;
            this->visualContacts[i].lastUpdated = SysClock::unixTime();
            contactExists = true;
        }
    }

    if(!contactExists)
    {
        VisualCollidable_t visualContact;
        visualContact.id = id;
        visualContact.bearing = bearing;
        visualContact.lastUpdated = SysClock::unixTime();

        this->visualContacts.push_back(visualContact);
    }

    this->visualListMutex.unlock();
    this->ownVisualLock = false;
}

///----------------------------------------------------------------------------------
CollidableList<AISCollidable_t> CollidableMgr::getAISContacts()
{
    return CollidableList<AISCollidable_t>(&this->aisListMutex, &aisContacts);
}

///----------------------------------------------------------------------------------
CollidableList<VisualCollidable_t> CollidableMgr::getVisualContacts()
{
    return CollidableList<VisualCollidable_t>(&this->visualListMutex, &visualContacts);
}

///----------------------------------------------------------------------------------
void CollidableMgr::removeOldContacts()
{
    if( !this->ownVisualLock )
    {
        this->visualListMutex.lock();
        this->ownVisualLock = true;
    }

    int timeNow = SysClock::unixTime();


    for (auto it = this->visualContacts.cbegin(); it != this->visualContacts.cend();)
    {
        if ( (*it).lastUpdated + VISUAL_CONTACT_TIME_OUT < timeNow )
        {
            it = visualContacts.erase(it);
        }
        else
        {
            ++it;
        }
    }

    this->visualListMutex.unlock();
    this->ownVisualLock = false;

    if( !this->ownAISLock )
    {
        this->aisListMutex.lock();
        this->ownAISLock = true;
    }

    timeNow = SysClock::unixTime();


    for (auto it = this->aisContacts.cbegin(); it != this->aisContacts.cend();)
    {
        if ( (*it).lastUpdated + AIS_CONTACT_TIME_OUT < timeNow )
        {
            it = aisContacts.erase(it);
        }
        else
        {
            ++it;
        }
    }

    this->aisListMutex.unlock();
    this->ownAISLock = false;
}

///----------------------------------------------------------------------------------
void CollidableMgr::ContactGC(CollidableMgr* ptr)
{
    while(true)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(std::min(AIS_CONTACT_TIME_OUT, VISUAL_CONTACT_TIME_OUT)));
        ptr->removeOldContacts();
    }
}
