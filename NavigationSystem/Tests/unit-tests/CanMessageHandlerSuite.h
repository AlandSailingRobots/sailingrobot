#pragma once

#include "../../Hardwares/CAN_Services/CanBusCommon/CanMessageHandler.h"

#include "../cxxtest/cxxtest/TestSuite.h"

class CanMessageHandlerSuite : public CxxTest::TestSuite {
   public:
    const int MESSAGE_ID = 500;

    void test_encodeAndDecode() {
        CanMessageHandler encodeHandler(MESSAGE_ID);

        int data = 3555;
        int dataMinValue = 0;
        int dataMaxValue = 4000;

        encodeHandler.encodeMessage(2, data);
        encodeHandler.encodeMappedMessage(3, data, dataMinValue, dataMaxValue);

        CanMessageHandler decodeHandler(encodeHandler.getMessage());

        int decodeData;
        double decodeData2;

        decodeHandler.getData(&decodeData, 2);
        decodeHandler.getMappedData(&decodeData2, 3, dataMinValue, dataMaxValue);

        bool floatIsInRange = (data < decodeData2 + 0.01) && (data > decodeData2 - 0.01);

        TS_ASSERT(data == decodeData && floatIsInRange &&
                  decodeHandler.getErrorMessage() == NO_ERRORS &&
                  decodeHandler.getMessageId() == MESSAGE_ID);
    }

    void test_encode_error() {
        CanMessageHandler messageHandler(MESSAGE_ID);

        messageHandler.encodeMappedMessage(4, 54333, 0, 50000);
        TS_ASSERT(messageHandler.getErrorMessage() == ERROR_CANMSG_DATA_OUT_OF_INTERVAL)
    }

    void test_encode_index_error() {
        CanMessageHandler messageHandler(MESSAGE_ID);

        messageHandler.encodeMappedMessage(4, 555, 0, 50000);
        messageHandler.encodeMappedMessage(4, 555, 0, 50000);
        TS_ASSERT(messageHandler.getErrorMessage() == ERROR_CANMSG_INDEX_OUT_OF_INTERVAL)
    }
};
