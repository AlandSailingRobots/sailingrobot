/****************************************************************************************
 *
 * File:
 * 		CollidableList.cpp
 *
 * Purpose:
 *		A list which a mutex associated with it. Prevents multiple sources from accessing
 *      the underlying vector unless it has locked the mutex.
 *
 * License:
 *      This file is subject to the terms and conditions defined in the file 
 *      'LICENSE.txt', which is part of this source code package.
 *
 ***************************************************************************************/


#include "CollidableList.h"


///----------------------------------------------------------------------------------
template<typename T>
CollidableList<T>::CollidableList( std::mutex* mutex, std::vector<T>* data )
    :ownsLock(false), index(0), listMutex(mutex), dataPtr(data)
{

}

///----------------------------------------------------------------------------------
template<typename T>
CollidableList<T>::~CollidableList()
{
    this->release();
}

///----------------------------------------------------------------------------------
template<typename T>
void CollidableList<T>::release()
{
    if(this->ownsLock)
    {
        this->listMutex->unlock();
        this->ownsLock = false;
    }
}

///----------------------------------------------------------------------------------
template<typename T>
T CollidableList<T>::next()
{
    if( !this->ownsLock )
    {
        this->listMutex.lock();
        this->ownsLock = true;
    }

    uint16_t length = this->length();

    T nextItem = dataPtr[index];

    if(this->index < this->length() - 1)
    {
        this.index++;
    }

    return nextItem;
}

///----------------------------------------------------------------------------------
template<typename T>
uint16_t CollidableList<T>::length()
{
    return dataPtr->size();
}