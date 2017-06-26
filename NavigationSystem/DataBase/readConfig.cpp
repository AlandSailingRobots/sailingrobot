#include "readConfig.h"

/*
* Parses the json file and puts the data in the json object cfg
*/
void readConfig::readFromJsonFile(const std::string& filename, json& cfg) {
  std::ifstream ifs(filename);
  cfg = json::parse(ifs);
}

/*
* Returns true if the key exists in cfg, and false if it doesn't
*/
bool readConfig::exists(json &cfg, const std::string &key){
  return cfg[key]!=NULL;
}

/*
* Returns the waypoints in the db in the json object wp
*/
void readConfig::waypointsInJson(json& wp, DBHandler db) {
  wp = db.getWaypoints();
}

/*
* Updates the database with the data in the file
* (Reads the file and puts it in the database)
*/
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

/*
* Private help function to add "id": "1" to the json
* Needs to be there, because the "id" field is required
* for the dbHandler functions to work
*/
json readConfig::merge(const json &a, const json &b) {
    json result = a.flatten();
    json tmp = b.flatten();

    for (json::iterator it = tmp.begin(); it != tmp.end(); ++it)
    {
        result[it.key()] = it.value();
    }

    return result.unflatten();
}
