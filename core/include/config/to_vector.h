#pragma once

#include <rapidjson/rapidjson.h>
#include <rapidjson/document.h>

#include <vector>

template <typename T>
std::vector<T> to_vector(const rapidjson::Value& val) {
  auto arr = val.GetArray();
  std::vector<T> res;
  res.reserve(arr.Size());
  for (const auto& el : arr) {
    res.push_back(el.template Get<T>());
  }
  return res;
}
