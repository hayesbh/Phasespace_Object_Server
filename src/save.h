// File: save.h
// Author: Dylan Visher
// Date: 5/18/13
// About: Header file for saving and loading objects

// Include the ROS system
#include <ros/ros.h>
#include <sys/stat.h>
#include <sys/types.h>
// C++ includes
#include <string>
#include <vector>
#include <algorithm>
#include <fstream>

// JSON rapidjson
#include <rapidjson/writer.h>
#include <rapidjson/prettywriter.h>
#include <rapidjson/filestream.h>
#include <rapidjson/document.h>
#include <rapidjson/stringbuffer.h>

// Object functionality
#include "PhaseSpace/Object.h"
#include "PhaseSpace/PSObject.h"
#include "PhaseSpace/ManualObject.h"
#include "PhaseSpace/Point.h"

namespace object_server {

using std::vector;
using std::string;
using std::stringstream;

bool store_object(Object* obj, string filename);
bool store_env(string env_ext, string env_name, vector<Object*> objects);
bool revive_object(string filename, vector<int>& ids_set_, Object** object, int object_count);
bool restore_env(string env_ext, string env_name, vector<int>& ids_set_, vector<Object*>& objects, int &object_count);

}
