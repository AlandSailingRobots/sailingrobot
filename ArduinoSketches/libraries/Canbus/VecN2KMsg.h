
#ifndef VECN2KMSG_H
#define VECN2KMSG_H

#include "MsgParsing.h"

class VecN2kMsg  // circular buffer
{
   public:
    bool PushBack(N2kMsgArd NMsg);
    bool PopFront(N2kMsgArd NMsg);

    N2kMsgArd Data[100];
    int Num = 0;
    int In = 0;
    int Out = 0;
};

#endif
