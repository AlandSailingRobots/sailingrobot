#include "readConfig.h"

void readConfig::readFromJsonFile(const std::string& filename, json& cfg) {
  std::ifstream ifs(filename);
  cfg = json::parse(ifs);
}

//void readConfig::jsonUnparsed(const std::string& filename, std::ifstream& unp) {
//  std::ifstream unp(filename);
//}

bool readConfig::exists(json &cfg, const std::string &key){
  return cfg[key]!=NULL;
}
/* */
void readConfig::waypointsInJson(json& wp, DBHandler db) {
  wp = db.getWaypoints();
}

// */
