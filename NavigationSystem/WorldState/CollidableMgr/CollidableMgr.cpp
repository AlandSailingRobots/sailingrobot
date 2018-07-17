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
 *
 * License:
 *      This file is subject to the terms and conditions defined in the file
 *      'LICENSE.txt', which is part of this source code package.
 *
 ***************************************************************************************/

#include "CollidableMgr.h"
#include "../SystemServices/SysClock.h"
#include "../SystemServices/Logger.h"
#include "../Math/Utility.h"
#include <chrono>


#define AIS_CONTACT_TIME_OUT        600        // 10 Minutes
#define NOT_AVAILABLE               -2000
#define LOOP_TIME                   1000

const unsigned int visualFieldFadeOutStart = 10;   
const unsigned int visualFieldTimeOut = 30; 
const int fadeOut = 2;  
///----------------------------------------------------------------------------------
CollidableMgr::CollidableMgr()
    :ownAISLock(false)
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
void CollidableMgr::addVisualField( std::map<int16_t, uint16_t> relBearingToRelObstacleDistance, int16_t heading)
{
 
    std::lock_guard<std::mutex> guard(m_visualMutex);
    auto updateTime = SysClock::unixTime();
    int lowBearing = 0;
    int highBearing = 0;
    for (auto it : relBearingToRelObstacleDistance){
        auto absBearing = Utility::limitAngleRange(it.first + heading); 
        m_visualField.bearingToRelativeObstacleDistance[absBearing] = it.second;
        m_visualField.bearingToLastUpdated[absBearing] = updateTime;
        if (it.first < lowBearing){
            lowBearing = it.first;
        }
        if (it.first > highBearing){
            highBearing = it.first;
        }
    }
    m_visualField.visualFieldLowBearing = Utility::limitAngleRange(lowBearing + heading);
    m_visualField.visualFieldHighBearing = Utility::limitAngleRange(highBearing + heading);
}

///----------------------------------------------------------------------------------
CollidableList<AISCollidable_t> CollidableMgr::getAISContacts()
{
    return CollidableList<AISCollidable_t>(&this->aisListMutex, &aisContacts);
}

///----------------------------------------------------------------------------------
VisualField_t CollidableMgr::getVisualField()
{
    std::lock_guard<std::mutex> guard(m_visualMutex);
    return m_visualField;
}

void CollidableMgr::removeOldVisualField(){
    std::lock_guard<std::mutex> guard(m_visualMutex);
    if (m_visualField.bearingToRelativeObstacleDistance.empty()){
        return;
    }
    std::vector<int16_t> eraseBearings;
    for (auto it : m_visualField.bearingToLastUpdated){
        if (it.second + visualFieldTimeOut < SysClock::unixTime()){
            eraseBearings.push_back(it.first);
        }
        else if (it.second + visualFieldFadeOutStart < SysClock::unixTime()){
            if (m_visualField.bearingToRelativeObstacleDistance[it.first] < 100){
                m_visualField.bearingToRelativeObstacleDistance[it.first] = 
                    std::min(m_visualField.bearingToRelativeObstacleDistance[it.first] + fadeOut, 100);
            }
        }
    }
    for (auto it :eraseBearings){
        Logger::info("erasing field for bearing: %d", it);
        m_visualField.bearingToRelativeObstacleDistance.erase(it);
        m_visualField.bearingToLastUpdated.erase(it);
    } 
   
}

///----------------------------------------------------------------------------------
void CollidableMgr::removeOldAISContacts()
{
 
    if( !this->ownAISLock )
    {
        this->aisListMutex.lock();
        this->ownAISLock = true;
    }

    auto timeNow = SysClock::unixTime();


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
        std::this_thread::sleep_for(std::chrono::milliseconds(LOOP_TIME));
        ptr->removeOldAISContacts();
        ptr->removeOldVisualField();
    }
}


