
#ifndef SAILINGROBOT_CANMESSAGEHANDLER_H
#define SAILINGROBOT_CANMESSAGEHANDLER_H

#include <stdint.h>

#include "../Canbus/MsgParsing.h"
#include "Utility.h"

class CanMessageHandler {
private:
    const int MAX_DATA_INDEX = 6;
    const int INDEX_ERROR_CODE = 7;

    const int BITS_PER_BYTE = 8;

    int currentDataWriteIndex = 0;
    int currentDataReadIndex = 0;

    CanMsg m_message;
public:
    explicit CanMessageHandler(uint32_t messageId);

    explicit CanMessageHandler(CanMsg message);

    unsigned long int getData(int lengthInBytes);

    double getMappedData(int lengthInBytes, long int minValue, long int maxValue);

    void setErrorMessage(uint8_t errorMessage);

    CanMsg getMessage();

    uint8_t getError();

    template<class T>
    bool encodeMessage(int lengthInBytes, T data) {
        for(int i=0;i<lengthInBytes;i++) {
            int dataIndex = currentDataWriteIndex+i;

            if(dataIndex > MAX_DATA_INDEX) {
                currentDataWriteIndex = dataIndex;
                return false;
            }
            m_message.data[dataIndex] = (data >> 8*i) &0xff;
        }
        currentDataWriteIndex += lengthInBytes;
        return true;
    }

    template<class T>
    bool encodeMappedMessage(int lengthInBytes, T data, long int minValue, long int maxValue) {
        auto possibilitiesDataCanHold = Utility::calcSizeOfBytes(lengthInBytes)-1;
        auto mappedData = static_cast<uint64_t>(
                Utility::mapInterval(data, minValue, maxValue, 0, possibilitiesDataCanHold));

        encodeMessage(lengthInBytes, mappedData);
        return true;
    }
};


#endif //SAILINGROBOT_CANMESSAGEHANDLER_H
