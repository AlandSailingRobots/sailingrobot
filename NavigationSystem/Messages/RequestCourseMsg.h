/****************************************************************************************
 *
 * File:
 * 		RequestCourseMsg.h
 *
 * Purpose:
 *
 * License:
 *      This file is subject to the terms and conditions defined in the file
 *      'LICENSE.txt', which is part of this source code package.
 *
 ***************************************************************************************/

#pragma once

#include "MessageBus/Message.h"

class RequestCourseMsg : public Message {
   public:
    RequestCourseMsg(NodeID destinationID, NodeID sourceID)
        : Message(MessageType::RequestCourse, sourceID, destinationID) {}

    RequestCourseMsg() : Message(MessageType::RequestCourse, NodeID::None, NodeID::None) {}

    RequestCourseMsg(MessageDeserialiser deserialiser) : Message(deserialiser) {}

    virtual ~RequestCourseMsg() {}
};
