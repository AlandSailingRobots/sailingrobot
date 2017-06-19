#include <string>
#include <stdint.h>
#include <iostream>
#include <fstream>

#include "libs/json/src/json.hpp"
#include "dbhandler/DBHandler.h"

using json = nlohmann::json;

class readConfig {
public:
  static void readFromJsonFile(const std::string& filename, json& cfg);
  static bool exists(json& cfg, const std::string& key);
  static void waypointsInJson(json& cfg, DBHandler db);
  static bool updateConfiguration(const std::string& file, DBHandler db);
private:
  static json merge(const json& a, const json& b);
};
