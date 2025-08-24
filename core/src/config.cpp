#include "config.h"

#include <fstream>

using namespace std;

nlohmann::json config;

void load_config() {
    ifstream config_file("config.json");
    config = nlohmann::json::parse(config_file);
    config_file.close();
}