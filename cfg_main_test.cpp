#include "readConfig.h"

int main() {
  //readConfig rc;
  const std::string& f2 = "configuraion.json";
  std::cout << f2 << '\n';
  json c2;
  readConfig::readFromJsonFile("configuration.json",c2);
  std::cout << c2 <<'\n';
  std::cout << c2["boat"] <<'\n';

  int idb = c2["install_db"];
  int udb = c2["update_db"];
  int i = idb + udb;
  std::cout << i << '\n';

  if (readConfig::exists(c2,"boat")) {
    std::cout << "ASPire " << c2["boat"]<<'\n';
  }

  json wp;
  readConfig::readFromJsonFile("course_1.json", wp);
  std::cout << wp <<'\n';
  std::cout << '\n'<<wp["traffic"].flatten() <<'\n';
  std::cout << wp.flatten()["/traffic/0/mmsi"] <<'\n';

  json test;
  readConfig::readFromJsonFile("test.json",test);
  std::cout << test << '\n';
  std::cout << test["age"]["days"]<<'\n';
}
