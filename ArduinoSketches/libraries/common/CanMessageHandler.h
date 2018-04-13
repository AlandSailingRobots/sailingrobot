/****************************************************************************************
*
* File:
* 		CanMessageHandler.h
*
* Purpose:
*		 The purpose of this class is a unified use of CanMsg handling
 *		 from both Arduino and RPI
 *
 *		 NOTE:
 *		 There is only 7 bytes of data that can be encoded by using this class,
 *		 because the last byte of the CanMsg is reserved for an error message.
*
* Developer Notes:
*
***************************************************************************************/

#ifndef SAILINGROBOT_CANMESSAGEHANDLER_H
#define SAILINGROBOT_CANMESSAGEHANDLER_H

#include <stdint.h>

#include "MsgParsing.h"
#include "CanUtility.h"

class CanMessageHandler {
private:
    const int MAX_DATA_INDEX = 6;
    const int INDEX_ERROR_CODE = 7;

    int currentDataWriteIndex = 0;
    int currentDataReadIndex = 0;

    CanMsg m_message;
public:

    /**
     * Class constructor
     *
     * Initializes a new clean CanMsg
     *
     * @param messageId the message id of CanMsg
     */
    explicit CanMessageHandler(uint32_t messageId);

    /**
     * Class constructor
     *
     * Initializes a Message handler for the CanMsg given to constructor
     *
     * @param message
     */
    explicit CanMessageHandler(CanMsg message);

    /**
     * Function to retrieve data from the CanMsg.
     * Class contains a internal index counter so there is no need to keep track of index positions
     *
     * @param lengthInBytes the number of bytes you want to retrieve
     * @return the value of the bytes accessed.
     */
    unsigned long int getData(int lengthInBytes);

    /**
     * Function to retrieve data from CanMsg and interpret
     * them to the value they had before they were inserted into CanMsg.
     *
     *
     * @param lengthInBytes the number of bytes you want to retrieve
     * @param minValue The lower part of the interval you want to interpret data to
     * @param maxValue The higher part of the interval you want to interpret data to
     * @return the decoded value
     */
    double getMappedData(int lengthInBytes, long int minValue, long int maxValue);

    /**
     * Inserts a value used as a error message.
     * Intended to be used with the error definitions in canbus_error_defs.h
     * @param errorMessage A value between 0 - 255
     */
    void setErrorMessage(uint8_t errorMessage);

    /**
     * Retrieves the constructed CanMsg from handler
     * @return the current CanMsg
     */
    CanMsg getMessage();

    /**
     * Get an value between 0 - 255 used as an error message
     * @return the inserted error message from CanMsg
     */
    uint8_t getError();

    /**
     * Encodes a clean positive integer value into canMsg.
     * Note: data value MUST be within the range of the lengthInBytes parameter
     *
     * Class contains an data index counter which is incremented after every insert
     * so no need of keeping track of index
     *
     * @tparam T The data type used, must be a positive integer value.
     * @param lengthInBytes The number of bytes this data requires
     * @param data The data that needs to be encoded into the CanMsg
     * @return false if there is no more room in CanMsg
     */
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

    /**
     * Encodes a value into canMsg, mapped onto the range given and the bytes available.
     * Note: A bigger range of data this leads to less precision
     *       A smaller amount of length leads to less precision
     *
     * Class contains an data index counter which is incremented after every insert
     * so no need of keeping track of index
     *
     * @tparam T The data type inserted
     * @param lengthInBytes The number of bytes this data requires
     * @param data The data that needs to be encoded into the CanMsg
     * @param minValue The lower part of the interval you want to encode data to
     * @param maxValue The higher part of the interval you want to encode data to
     * @return
     */
    template<class T>
    bool encodeMappedMessage(int lengthInBytes, T data, long int minValue, long int maxValue) {
        auto possibilitiesDataCanHold = CanUtility::calcSizeOfBytes(lengthInBytes)-1;
        auto mappedData = static_cast<uint64_t>(
                CanUtility::mapInterval(data, minValue, maxValue, 0, possibilitiesDataCanHold));

        encodeMessage(lengthInBytes, mappedData);
        return true;
    }
};


#endif //SAILINGROBOT_CANMESSAGEHANDLER_H
