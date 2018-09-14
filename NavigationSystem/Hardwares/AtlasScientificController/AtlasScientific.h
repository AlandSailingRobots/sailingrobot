#pragma once
#include <string>
#include <vector>
#include "Hardwares/i2ccontroller/I2CController.h"

class AtlasI2C : public I2CController {
   public:
    AtlasI2C() : I2CController(){};

    int ASWRITE(std::string cmd);
    std::string ASREAD();
    std::string ASQUERY(std::string text);
};
