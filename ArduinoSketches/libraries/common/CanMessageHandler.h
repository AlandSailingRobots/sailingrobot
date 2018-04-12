//
// Created by sailbot on 2018-04-12.
//

#ifndef SAILINGROBOT_CANMESSAGEHANDLER_H
#define SAILINGROBOT_CANMESSAGEHANDLER_H


class CanMessageHandler {
    template<class T>
    void encodeMessage(int startPosition, int length, T data);
};


#endif //SAILINGROBOT_CANMESSAGEHANDLER_H
