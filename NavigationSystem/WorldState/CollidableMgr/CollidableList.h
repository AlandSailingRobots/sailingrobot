/****************************************************************************************
 *
 * File:
 * 		CollidableList.h
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

#pragma once

#include <stdint.h>
#include <mutex>
#include <vector>

template <typename T>
class CollidableList {
   public:
    ///----------------------------------------------------------------------------------
    /// Constructs the collidable list, the data and the mutex to the data are passed in
    /// as arguments.
    ///----------------------------------------------------------------------------------
    CollidableList<T>(std::mutex* mutex, std::vector<T>* data)
        : ownsLock(false), index(0), listMutex(mutex), dataPtr(data) {}

    ///----------------------------------------------------------------------------------
    /// Frees the mutex if this list has locked it.
    ///----------------------------------------------------------------------------------
    ~CollidableList() { this->release(); }

    ///----------------------------------------------------------------------------------
    /// Frees the mutex over the list if this list has locked it.
    ///----------------------------------------------------------------------------------
    void release() {
        if (this->ownsLock) {
            this->listMutex->unlock();
            this->ownsLock = false;
        }
    }

    ///----------------------------------------------------------------------------------
    /// Returns the next data item in the underlying vector. If the end of the vector has
    /// been reached, the last item is returned. This function ensures it has locked the
    /// mutex associated with the underlying vector before attempting to access it.
    ///
    /// NOTE:
    ///     It does not release the lock, use either release() or destroy the object to
    ///     do this.
    ///----------------------------------------------------------------------------------
    T next() {
        if (!this->ownsLock) {
            this->listMutex->lock();
            this->ownsLock = true;
        }

        uint16_t length = this->length();

        T nextItem = dataPtr->at(index);

        if (this->index < length - 1) {
            this->index++;
        }

        return nextItem;
    }

    void reset() { this->index = 0; }

    ///----------------------------------------------------------------------------------
    /// Returns the number of items in the list.
    ///----------------------------------------------------------------------------------
    uint16_t length() { return dataPtr->size(); }

   private:
    bool ownsLock;
    uint16_t index;
    std::mutex* listMutex;
    std::vector<T>* dataPtr;
};