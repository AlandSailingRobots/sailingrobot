/****************************************************************************************
 *
 * File:
 * 		MessageBusTestHelper.h
 *
 * Purpose:
 *		A helper class to start a message bus asyncronously at construction,
 * and  to stop it at destruction
 *
 *
 ***************************************************************************************/

#pragma once

#include <future>
#include "MessageBus/MessageBus.h"

// Starts the message bus as an asyncrounous
class MessageBusTestHelper {
   private:
    static void runMessageLoop(MessageBus& messageBus) { messageBus.run(); };

    std::future<void> m_fut;
    MessageBus& m_messageBus;

   public:
    explicit MessageBusTestHelper(MessageBus& messageBus) : m_messageBus(messageBus) {
        m_fut = std::async(std::launch::async, runMessageLoop, std::ref(m_messageBus));
    };

    ~MessageBusTestHelper() {
        m_messageBus.stop();
        m_fut.get();
    };
};
