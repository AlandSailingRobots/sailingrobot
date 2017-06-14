#include "readConfig.h"

int main() {
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

  printf("\n");
  printf("\n");
  printf("\n");

  readConfig rc;
  std::string f2 = "configuraion.json";
  json c2;
  c2 = rc.readFromJsonFile(f2);
  std::cout << c2 <<'\n';

}
