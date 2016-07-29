#include <iostream>
#include "xmlparser/pugi/pugixml.hpp"

class Parse {

public:

  /* Parse waypoint information */
  static void waypoint(const char* filename) {
    pugi::xml_document doc;
    doc.load_file(filename);
    pugi::xml_node route = doc.child("route");
    for (pugi::xml_node wp = route.first_child(); wp; wp = wp.next_sibling()) {
      for (pugi::xml_node node = wp.first_child(); node; node = node.next_sibling()) {
        std::cout << node.name() << "=" << node.child_value() << std::endl;
      }
    }
  }

  /* Parse wind information */
  static void wind(const char* filename) {
    pugi::xml_document doc;
    doc.load_file(filename);
    pugi::xml_node wind = doc.child("wind");
    for (pugi::xml_node node = wind.first_child(); node; node = node.next_sibling()) {
        std::cout << node.name() << "=" << node.child_value() << std::endl;
    }
  }

  /* Parse configuration file */
  static void config(const char* filename) {
    pugi::xml_document doc;
    doc.load_file(filename);
    pugi::xml_node ship = doc.child("ship");
    for (pugi::xml_node child = ship.first_child(); child; child = child.next_sibling()) {
      for (pugi::xml_node node = child.first_child(); node; node = node.next_sibling()) {
        std::cout << node.name() << "=" << node.child_value() << std::endl;
      }
    }
  }

};

int main(void) {
  Parse::wind("xml/wind.xml");
  Parse::waypoint("xml/waypoint.xml");
  Parse::config("xml/config.xml");
}