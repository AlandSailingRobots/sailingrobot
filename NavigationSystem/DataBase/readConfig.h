#include <stdint.h>
#include <fstream>
#include <iostream>
#include <string>

#include "../Libs/json/src/json.hpp"
#include "DBHandler.h"

using json = nlohmann::json;

class readConfig {
   public:
    /*
     * Parses the json file and puts the data in the json object cfg
     */
    static void readFromJsonFile(const std::string& filename, json& cfg);

    /*
     * Returns true if the key exists in cfg, and false if it doesn't
     */
    static bool exists(json& cfg, const std::string& key);

    /*
     * Returns the waypoints in the db in the json object wp
     */
    static void waypointsInJson(json& cfg, DBHandler db);

    /*
     * Updates the database with the data in the file
     * (Reads the file and puts it in the database)
     */
    static bool updateConfiguration(const std::string& file, DBHandler db);

   private:
    /*
     * Private help function to add "id": "1" to the json
     * Needs to be there, because the "id" field is required
     * for the dbHandler functions to work
     */
    static json merge(const json& a, const json& b);
};
