#pragma once

#include "Nodes/Node.h"
#include "Messages/WindStateMsg.h"
#include "Messages/NavigationControlMsg.h"
#include "Messages/MessageTypes.h"
#include "MessageBus/MessageBus.h"

class DBLoggerNode: public Node {
public:
    DBLoggerNode();

    void processMessage(const Message* message);

private:

};