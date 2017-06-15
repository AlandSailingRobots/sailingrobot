#include "readConfig.h"

void readConfig::readFromJsonFile(const std::string& filename, json& cfg) {
  std::ifstream ifs(filename);
  cfg = json::parse(ifs);
}

bool readConfig::exists(json &cfg, const std::string &key){
  return cfg[key]!=NULL;
}
