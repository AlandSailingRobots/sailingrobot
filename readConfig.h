#pragma once

#include "libs/json/src/json.hpp"
#include <string>
#include <stdint.h>
#include <iostream>
#include <fstream>
using json = nlohmann::json;

struct jsonString {
  std::string key;
  std::string value;
};

struct jsonInt {
  std::string key;
  int value;
};

struct jsonFloat {
  std::string key;
  float value;
};

class readConfig {
public:
  //readCofig();
  static json readFromJsonFile(const std::string& filename);


};
