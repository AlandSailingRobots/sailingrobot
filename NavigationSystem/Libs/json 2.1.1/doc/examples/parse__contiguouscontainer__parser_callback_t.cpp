#include "json.hpp"
#include <iomanip> // for std::setw

using json = nlohmann::json;

int main()
{
    // a JSON text given as std::vector
    std::vector<uint8_t> text = {'[', '1', ',', '2', ',', '3', ']', '\0'};

    // parse and serialize JSON
    json j_complete = json::parse(text);
    std::cout << std::setw(4) << j_complete << "\n\n";
}
