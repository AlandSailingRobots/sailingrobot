/****************************************************************************************
 *
 * File:
 * 		CollidableMgr.cpp
 *
 * Purpose:
 *
 * License:
 *      This file is subject to the terms and conditions defined in the file 
 *      'LICENSE.txt', which is part of this source code package.
 *
 ***************************************************************************************/


 #include "CollidableMgr.h"
 #include "SystemServices/SysClock.h"
 #include "SystemServices/Logger.h"


///----------------------------------------------------------------------------------
CollidableMgr::CollidableMgr()
    :ownAISLock(false), ownVisualLock(false)
{

}

///----------------------------------------------------------------------------------
void CollidableMgr::addAISContact( uint32_t mmsi, float lat, float lon, float speed, uint16_t course )
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