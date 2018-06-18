
#include "VecN2KMsg.h"

bool
VecN2kMsg::PushBack(N2kMsgArd NMsg)
{
    if(Num != 100)
    {
        ++Num;
        Data[In++] = NMsg;
        if(In == 100)
            In = 0;
        return true;
    }
    else
        return false;
}
bool
VecN2kMsg::PopFront(N2kMsgArd NMsg)
{
    if(Num != 0)
    {
        --Num;
        Data[Out++] = NMsg;
        if(Out == 100)
            Out = 0;
        return true;
    }
    else
        return false;
}