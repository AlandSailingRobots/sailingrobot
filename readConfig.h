//#pragma once

#include "libs/json/src/json.hpp"
#include <string>
#include <stdint.h>
#include <iostream>
#include <fstream>
using json = nlohmann::json;

class readConfig {
public:
  static void readFromJsonFile(const std::string& filename, json& cfg);
  static bool exists(json& cfg, const std::string& key);
};
