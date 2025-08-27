#include "utils/config.h"

#include <rapidjson/istreamwrapper.h>

#include <fstream>

using namespace std;
using namespace rapidjson;

Document config;

void load_config() {
    ifstream file("config.json");
    IStreamWrapper isw(file);
    config.ParseStream(isw);
    file.close();
}

vector<int> make_int_vector(const GenericArray<false, Value> &arr) {
    vector<int> res(arr.Size());
    for (int i = 0; i < arr.Size(); ++i) {
        res[i] = arr[i].GetInt();
    }
    return res;
}