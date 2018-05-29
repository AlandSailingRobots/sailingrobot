#ifndef canbus__h
#define canbus__h

#include <SPI.h>

#include <canbus_defs.h>
#include "MsgParsing.h"
#include "mcp2515.h"

class CanbusClass {
   private:
    void setFilterFromIndex(int filterIndex,
                            uint8_t& FSIDH,
                            uint8_t& FSIDL,
                            uint8_t& FEID8,
                            uint8_t& FEID0);

   public:
    CanbusClass(){};
    bool Init(int chipSelectPin);
    uint8_t CheckForMessages();

    bool SendMessage(CanMsg* Msg);
    bool GetMessage(CanMsg* Msg);

    void SetNormalMode();
    void SetSleepMode();
    void SetLoopBackMode();
    void SetListenOnlyMode();
    void SetConfigMode();
    void SetMasksAndActivateFilters(uint32_t Mask1, uint32_t Mask2, bool RollOver);
    void SetFilter(int FilterIndex, uint32_t Filter);
    void SetFilterAndMask(int ReceiveBuffer, int FilterIndex, uint32_t Filter, uint32_t Mask);
};

#endif
