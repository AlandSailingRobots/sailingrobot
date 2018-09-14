/****************************************************************************************
 *
 * File:
 * 		ASRCourseBallotSuite.h
 *
 * Purpose:
 *
 * License:
 *      This file is subject to the terms and conditions defined in the file
 *      'LICENSE.txt', which is part of this source code package.
 *
 * Developer Notes:
 *
 *							11.4.17 JM
 *
 *	Functions that have tests:		Functions that does not have tests:
 *
 *	set 							clear
 *	ptr								get
 *	add 							maxVotes
 *
 ***************************************************************************************/

#pragma once

#include "Navigation/LocalNavigationModule/ASRCourseBallot.h"
#include "Tests/cxxtest/cxxtest/TestSuite.h"

class ASRCourseBallotSuite : public CxxTest::TestSuite {
   public:
    ///----------------------------------------------------------------------------------
    void test_ASRCourseBallot_set() {
        ASRCourseBallot ballot(360);

        for (int i = 0; i < ASRCourseBallot::ELEMENT_COUNT; i++) {
            ballot.set(i, ASRCourseBallot::ELEMENT_COUNT - i);
        }

        const int16_t* ptr = ballot.ptr();
        for (int i = 0; i < ASRCourseBallot::ELEMENT_COUNT; i++) {
            TS_ASSERT_EQUALS(ptr[i], ASRCourseBallot::ELEMENT_COUNT - i);
        }
    }

    ///----------------------------------------------------------------------------------
    void test_ASRCourseBallot_set_Angle() {
        ASRCourseBallot ballot(360);
        const int16_t* ptr = ballot.ptr();

        ballot.set(370, ASRCourseBallot::ELEMENT_COUNT);
        TS_ASSERT_EQUALS(ptr[10], ASRCourseBallot::ELEMENT_COUNT);

        ballot.set(-90, ASRCourseBallot::ELEMENT_COUNT);
        TS_ASSERT_EQUALS(ptr[270], ASRCourseBallot::ELEMENT_COUNT);
    }

    ///----------------------------------------------------------------------------------
    void test_ASRCourseBallot_set_MaxVote() {
        ASRCourseBallot ballot(5);

        for (int i = 0; i < ASRCourseBallot::ELEMENT_COUNT; i++) {
            ballot.set(i, 5 + i);
        }

        const int16_t* ptr = ballot.ptr();
        for (int i = 0; i < ASRCourseBallot::ELEMENT_COUNT; i++) {
            TS_ASSERT_EQUALS(ptr[i], 5);
        }
    }

    ///----------------------------------------------------------------------------------
    void test_ASRCourseBallot_add() {
        ASRCourseBallot ballot(360);

        for (int i = 0; i < ASRCourseBallot::ELEMENT_COUNT; i++) {
            ballot.add(i, 1);
        }

        const int16_t* ptr = ballot.ptr();
        for (int i = 0; i < ASRCourseBallot::ELEMENT_COUNT; i++) {
            TS_ASSERT_EQUALS(ptr[i], 1);
        }

        for (int i = 0; i < ASRCourseBallot::ELEMENT_COUNT; i++) {
            ballot.add(i, 1);
        }

        for (int i = 0; i < ASRCourseBallot::ELEMENT_COUNT; i++) {
            TS_ASSERT_EQUALS(ptr[i], 2);
        }
    }

    ///----------------------------------------------------------------------------------
    void test_ASRCourseBallot_add_Angle() {
        ASRCourseBallot ballot(360);
        const int16_t* ptr = ballot.ptr();

        ballot.add(370, ASRCourseBallot::ELEMENT_COUNT);
        TS_ASSERT_EQUALS(ptr[10], ASRCourseBallot::ELEMENT_COUNT);

        ballot.add(-90, ASRCourseBallot::ELEMENT_COUNT);
        TS_ASSERT_EQUALS(ptr[270], ASRCourseBallot::ELEMENT_COUNT);
    }

    ///----------------------------------------------------------------------------------
    void test_ASRCourseBallot_add_MaxVote() {
        ASRCourseBallot ballot(5);

        for (int i = 0; i < ASRCourseBallot::ELEMENT_COUNT; i++) {
            ballot.add(i, 6);
        }

        const int16_t* ptr = ballot.ptr();
        for (int i = 0; i < ASRCourseBallot::ELEMENT_COUNT; i++) {
            TS_ASSERT_EQUALS(ptr[i], 5);
        }

        for (int i = 0; i < ASRCourseBallot::ELEMENT_COUNT; i++) {
            ballot.add(i, 6);
        }

        for (int i = 0; i < ASRCourseBallot::ELEMENT_COUNT; i++) {
            TS_ASSERT_EQUALS(ptr[i], 5);
        }
    }

    uint16_t ASRCourseBallot_mock_calculateIndex(uint16_t heading, int courseRes) {
        return heading / courseRes;
    }

    ///----------------------------------------------------------------------------------
    void test_ASRCourseBallot_courseIndex() {
        TS_ASSERT_EQUALS(ASRCourseBallot_mock_calculateIndex(0, 1), 0);
        TS_ASSERT_EQUALS(ASRCourseBallot_mock_calculateIndex(5, 1), 5);
        TS_ASSERT_EQUALS(ASRCourseBallot_mock_calculateIndex(200, 1), 200);
        TS_ASSERT_EQUALS(ASRCourseBallot_mock_calculateIndex(359, 1), 359);

        TS_ASSERT_EQUALS(ASRCourseBallot_mock_calculateIndex(0, 2), 0);
        TS_ASSERT_EQUALS(ASRCourseBallot_mock_calculateIndex(5, 2), 2);
        TS_ASSERT_EQUALS(ASRCourseBallot_mock_calculateIndex(200, 2), 100);
        TS_ASSERT_EQUALS(ASRCourseBallot_mock_calculateIndex(359, 2), 179);

        TS_ASSERT_EQUALS(ASRCourseBallot_mock_calculateIndex(0, 3), 0);
        TS_ASSERT_EQUALS(ASRCourseBallot_mock_calculateIndex(5, 3), 1);
        TS_ASSERT_EQUALS(ASRCourseBallot_mock_calculateIndex(200, 3), 66);
        TS_ASSERT_EQUALS(ASRCourseBallot_mock_calculateIndex(359, 3), 119);

        TS_ASSERT_EQUALS(ASRCourseBallot_mock_calculateIndex(0, 4), 0);
        TS_ASSERT_EQUALS(ASRCourseBallot_mock_calculateIndex(5, 4), 1);
        TS_ASSERT_EQUALS(ASRCourseBallot_mock_calculateIndex(200, 4), 50);
        TS_ASSERT_EQUALS(ASRCourseBallot_mock_calculateIndex(359, 4), 89);
    }
};