#pragma once

#include <rapidjson/document.h>

#include <vector>

extern rapidjson::Document config;

void load_config();

std::vector<int> make_int_vector(
    const rapidjson::GenericArray<false, rapidjson::Value> &);