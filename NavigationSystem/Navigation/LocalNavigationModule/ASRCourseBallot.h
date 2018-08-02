/****************************************************************************************
 *
 * File:
 * 		ASRCourseBallot.h
 *
 * Purpose:
 *		The Course Ballot is used by the voters and the Arbiter to store the possible
 *      course headings and their votes. It provides functions for setting and clearing
 *      the voting scores, as well as accessing the underlying structure.
 *
 * License:
 *      This file is subject to the terms and conditions defined in the file
 *      'LICENSE.txt', which is part of this source code package.
 *
 ***************************************************************************************/

#pragma once

#include <stdint.h>
#include <cstring>
#include <algorithm>
#include <iterator>

class ASRCourseBallot {
    friend class ASRArbiter;

   public:
    ///----------------------------------------------------------------------------------
    /// Constructs a CourseBallot. Requires a single argument which controls the maximum
    /// vote a single course can have.
    ///----------------------------------------------------------------------------------
    ASRCourseBallot(int16_t maxVotes);

    ///----------------------------------------------------------------------------------
    /// Assigns a vote to a particular heading.
    ///----------------------------------------------------------------------------------
    void set(uint16_t course, int16_t value);

    ///----------------------------------------------------------------------------------
    /// Assigns a veto to a particular heading.
    /// NOTE: Can only set a veto, no possibilities to unset one.
    ///----------------------------------------------------------------------------------
    void setVeto(uint16_t course);

    ///----------------------------------------------------------------------------------
    /// Adds a vote to a particular heading.
    ///----------------------------------------------------------------------------------
    void add(uint16_t course, int16_t value);

    ///----------------------------------------------------------------------------------
    /// Resets the ballot, clearing all set votes.
    ///----------------------------------------------------------------------------------
    void clear();

    ///----------------------------------------------------------------------------------
    /// Gets the votes placed on a heading. The heading is rounded down to the nearest
    /// valid heading;
    ///----------------------------------------------------------------------------------
    int16_t get(uint16_t heading) const;

    ///----------------------------------------------------------------------------------
    /// Check if a veto is placed on a heading;
    ///----------------------------------------------------------------------------------
    bool getVeto(uint16_t heading) const;

    ///----------------------------------------------------------------------------------
    /// Returns a pointer to the underlying course data, this is an array that has
    /// 360 / CourseBallot::COURSE_RESOLUTION elements.
    ///----------------------------------------------------------------------------------
    const int16_t* ptr() const;

    int16_t maxVotes() const { return MAX_VOTES; };

    ///----------------------------------------------------------------------------------
    /// The course resolution controls the number of courses that the boat will examine.
    /// The number of courses examined is 360 / CourseBallot::COURSE_RESOLUTION. This
    /// should be some value that 360 is divisible by without resulting in a fraction.
    ///----------------------------------------------------------------------------------
    static const int COURSE_RESOLUTION = 1;

    ///----------------------------------------------------------------------------------
    /// The number of courses the ballot tracks.
    ///----------------------------------------------------------------------------------
    static const int ELEMENT_COUNT = 360 / COURSE_RESOLUTION;

    ///----------------------------------------------------------------------------------
    /// Find min and max, useful for debugging, tuning
    ///----------------------------------------------------------------------------------
    std::pair<int , int > getMin() {
        std::pair<int, int> PAIR;
        int16_t* p_index = std::min_element(std::begin(courses), std::end(courses));
        int index = std::distance(std::begin(courses), p_index);
        PAIR.first = index;
        PAIR.second = *p_index;
        return PAIR;
    };
    std::pair<int , int > getMax() {
        std::pair<int, int> PAIR;
        int16_t* p_index = std::max_element(std::begin(courses), std::end(courses));
        const int index = std::distance(std::begin(courses),p_index);
        PAIR.first = index;
        PAIR.second = *p_index;
        return PAIR;
    };

// TODO: Put back as a private variable after debugging
    int16_t courses[ELEMENT_COUNT];
    bool veto[ELEMENT_COUNT];
   private:
    const int16_t MAX_VOTES;
    


};