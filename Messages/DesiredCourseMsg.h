/****************************************************************************************
 *
 * File:
 * 		DesiredCourseMsg
 .h
 *
 * Purpose:
 *
 * License:
 *      This file is subject to the terms and conditions defined in the file 
 *      'LICENSE.txt', which is part of this source code package.
 *
 ***************************************************************************************/


#pragma once


#include "Message.h"


class DesiredCourseMsg : public Message {
public:
	DesiredCourseMsg( NodeID destinationID, NodeID sourceID, uint16_t desiredCourse )
		:Message( MessageType::DesiredCourse, sourceID, destinationID ), course( desiredCourse )
	{ }

	DesiredCourseMsg( uint16_t desiredCourse )
		:Message( MessageType::DesiredCourse, NodeID::None, NodeID::None ), course( desiredCourse )
	{ }

	DesiredCourseMsg( MessageDeserialiser deserialiser )
		:Message( deserialiser )
	{ }

	virtual ~DesiredCourseMsg() { }

	uint16_t desiredCourse() { return course; }

private:
	uint16_t course;
};
