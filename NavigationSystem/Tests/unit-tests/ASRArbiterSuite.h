/****************************************************************************************
 *
 * File:
 * 		ASRArbiterSuite.h
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
 *	Functions that have tests:		Functions that does not have tests:
 *	
 *	castVote
 *	clearBallot
 *	getWinner
 *	getResult
 *
 ***************************************************************************************/

 #pragma once

 #include "../cxxtest/cxxtest/TestSuite.h"
 #include "Navigation/LocalNavigationModule/ASRArbiter.h"


 class ASRArbiterSuite : public CxxTest::TestSuite {
	public:

	///----------------------------------------------------------------------------------
	void test_ASRArbiter_vote()
	{
		ASRArbiter arbiter;
		ASRCourseBallot ballot( 10 );

		for( int i = 0; i < ASRCourseBallot::ELEMENT_COUNT; i++ )
		{
			ballot.set( i, 10 );
		}

		ASRCourseBallot ballot2( 10 );

		for( int i = 0; i < ASRCourseBallot::ELEMENT_COUNT; i++ )
		{
			if( i % 2 == 0)
			{
				ballot2.set( i, 2 );
			}
			else 
			{
				ballot2.set( i, 1 );
			}
		}

		arbiter.clearBallot();
		arbiter.castVote(ballot);
		arbiter.castVote(ballot2);

		const ASRCourseBallot& ptr = arbiter.getResult();

		for( int i = 0; i < ASRCourseBallot::ELEMENT_COUNT; i++ )
		{
			if( i % 2 == 0)
			{
				TS_ASSERT_EQUALS( ptr.get( i ), 12 );
			}
			else 
			{
				TS_ASSERT_EQUALS( ptr.get( i ), 11 );
			}
		}

	}

	///----------------------------------------------------------------------------------
	void test_ASRArbiter_getWinner()
	{
		ASRArbiter arbiter;
		ASRCourseBallot ballot( 10 );

		for( int i = 0; i < ASRCourseBallot::ELEMENT_COUNT; i++ )
		{
			ballot.set( i, 10 );
		}

		ASRCourseBallot ballot2( 10 );
		ballot2.set( 150, 2 );

		arbiter.clearBallot();
		arbiter.castVote(ballot);
		arbiter.castVote(ballot2);

		TS_ASSERT_EQUALS( arbiter.getWinner(), 150 );
	}

	///----------------------------------------------------------------------------------
	void test_ASRArbiter_clearBallot()
	{
		ASRArbiter arbiter;
		ASRCourseBallot ballot( 10 );

		for( int i = 0; i < ASRCourseBallot::ELEMENT_COUNT; i++ )
		{
			ballot.set( i, 10 );
		}

		arbiter.castVote(ballot);
		arbiter.clearBallot();

		const ASRCourseBallot& ptr = arbiter.getResult();

		for( int i = 0; i < ASRCourseBallot::ELEMENT_COUNT; i++ )
		{
			TS_ASSERT_EQUALS( ptr.get( i ), 0 );
		}
	}
 };