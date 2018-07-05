/****************************************************************************************
 *
 * File:
 * 		SysClock.h
 *
 * Purpose:
 *		Provides a number of time keeping functions. Can use the GPS to ensure a accurate
 *		time. The Sysclock will use the system time if a GPS time has not been provided.
 *		Timestamps are in GMT(UTC) time.
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/

#pragma once

#include <string>
//#include <sys/types.h>
//#include <sys/stat.h>

#define NEVER_UPDATED (unsigned int)(-1)

struct TimeStamp {
    TimeStamp() : unixTime(0), milliseconds(0) {}
    TimeStamp(unsigned long uTime, int mSec) : unixTime(uTime), milliseconds(mSec) {}

    unsigned long unixTime;
    unsigned int milliseconds;
};

class SysClock {
   public:
    ///----------------------------------------------------------------------------------
    /// Sets the unix time starting point, the SysClock will reset its internal unix time
    /// to this value.
    ///----------------------------------------------------------------------------------
    static void setTime(unsigned long unixTime);

    ///----------------------------------------------------------------------------------
    /// Returns the Unix time.
    ///----------------------------------------------------------------------------------
    static unsigned long unixTime();

    ///----------------------------------------------------------------------------------
    /// Returns the current millisecond
    ///----------------------------------------------------------------------------------
    static unsigned int millis();

    ///----------------------------------------------------------------------------------
    /// Returns a string representation of the current time in the format:
    ///			yyyy-mm-dd hh:mm:ss.fff
    ///----------------------------------------------------------------------------------
    static std::string timeStampStr();

    ///----------------------------------------------------------------------------------
    /// Returns the current time stamp.
    ///----------------------------------------------------------------------------------
    static TimeStamp timeStamp();

    ///----------------------------------------------------------------------------------
    /// Returns a string representation of the current time in the format:
    ///			hh:mm:ss
    ///----------------------------------------------------------------------------------
    static std::string hh_mm_ss();

    ///----------------------------------------------------------------------------------
    /// Returns a string representation of the current time in the format:
    ///			hh:mm:ss:ms
    ///----------------------------------------------------------------------------------
    static std::string hh_mm_ss_ms();
    static std::string hh_mm_ss_ms(TimeStamp timeStamp);

    ///----------------------------------------------------------------------------------
    /// Returns the current day of the month as an integer
    ///----------------------------------------------------------------------------------
    static int day();

    ///----------------------------------------------------------------------------------
    /// Returns the current month of the year as an integer
    ///----------------------------------------------------------------------------------
    static int month();

    ///----------------------------------------------------------------------------------
    /// Returns the current year as an integer
    ///----------------------------------------------------------------------------------
    static int year();

    ///----------------------------------------------------------------------------------
    /// Returns how long ago the SysCock time was last updated in seconds. The max value
    /// the unsigned int can hold indicates that it has never been updated, you can use
    /// NEVER_UPDATED to check for this.
    ///----------------------------------------------------------------------------------
    static unsigned int lastUpdated();

   private:
    static unsigned long m_LastUpdated;
    static unsigned long m_LastTimeStamp;
    static unsigned long m_LastClockTime;
};
