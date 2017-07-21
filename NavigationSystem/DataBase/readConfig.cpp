#include "readConfig.h"

void readConfig::readFromJsonFile(const std::string& filename, json& cfg) {
  std::ifstream ifs(filename);
  cfg = json::parse(ifs);
}

bool readConfig::exists(json &cfg, const std::string &key){
  return cfg[key]!=NULL;
}

void readConfig::waypointsInJson(json& wp, DBHandler db) {
  wp = db.getWaypoints();
}

bool readConfig::updateConfiguration(const std::string& file, DBHandler db) {
  json cfg;
  readFromJsonFile(file, cfg);
  json idconst = {{"id", "1"}};
  for (auto it = cfg.begin(); it != cfg.end(); ++it) {
    if (!exists(cfg[it.key()], "id")) {
      cfg[it.key()] = merge(cfg[it.key()],idconst);
    }
  }
  db.updateConfigs(cfg.dump());
  return true;
}

json readConfig::merge(const json &a, const json &b) {
    json result = a.flatten();
    json tmp = b.flatten();

    for (json::iterator it = tmp.begin(); it != tmp.end(); ++it)
    {
        result[it.key()] = it.value();
    }

    return result.unflatten();
}
