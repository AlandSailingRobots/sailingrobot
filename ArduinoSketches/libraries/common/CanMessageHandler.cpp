/****************************************************************************************
*
* File:
* 		CanMessageHandler.cpp
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


#include "canbus_error_defs.h"
#include "CanMessageHandler.h"

CanMessageHandler::CanMessageHandler(CanMsg message) : m_message(message){

}


CanMessageHandler::CanMessageHandler(uint32_t messageId) {
    m_message.id = messageId;
    m_message.header.ide = 0;
    m_message.header.length = 7;
    for(auto& byteData : m_message.data) {
        byteData = 0;
    }

    m_message.data[INDEX_ERROR_CODE] = NO_ERRORS;
}

unsigned long int CanMessageHandler::getData(int lengthInBytes) {

    unsigned long int data = 0;
    for (int i=0;i<lengthInBytes;i++) {
        if(currentDataReadIndex > MAX_DATA_INDEX) {
            return 0;
        }
        data += static_cast<unsigned long int>(m_message.data[currentDataReadIndex+i] << i*8);
    }
    currentDataReadIndex += lengthInBytes;
    return data;
}

double CanMessageHandler::getMappedData(int lengthInBytes, long int minValue, long int maxValue) {

    unsigned long int data = getData(lengthInBytes);
    auto possibilitiesDataCanHold = CanUtility::calcSizeOfBytes(lengthInBytes)-1;
    return CanUtility::mapInterval(data, 0, possibilitiesDataCanHold, minValue, maxValue);
}

void CanMessageHandler::setErrorMessage(uint8_t errorMessage) {
    m_message.data[INDEX_ERROR_CODE] = errorMessage;
}

CanMsg CanMessageHandler::getMessage() {
    return m_message;
}

uint8_t CanMessageHandler::getError() {
    return m_message.data[INDEX_ERROR_CODE];
}

