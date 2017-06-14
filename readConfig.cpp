
#include "readConfig.h"

//readConfig::readConfig() {

//}

json readFromJsonFile(const std::string& filename) {
  std::ifstream ifs(filename);
  json config = json::parse(ifs);
  std::cout << config << '\n';
  std::cout << filename << '\n';
  return config;
}

/* */
int main(int argc, char *argv[]) {
  std::string file = "configuration.json";
  std::cout << "HEJ\n";
  json config = readFromJsonFile(file);
  std::cout << config << '\n';
  return 1;
}
// */


  //----- json testing----------
  //using json = nlohmann::json;
  //json config;
/*
  std::ifstream ifs("configuration.json");
  json config = json::parse(ifs);
  std::cout << config.at("boat") << '\n';
  std::cout << config.at("update_db") << '\n';
  std::cout << config << '\n';
  //ifs >> config;
  //printf(config["boat"]);
  if(config.at("install_db")==1) {
    //system(install_db.sh);
    std::cout << "HEJ";
  }
*/
