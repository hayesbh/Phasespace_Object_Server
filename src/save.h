// File: save.h
// Author: Dylan Visher
// Date: 5/18/13
// About: Header file for saving and loading objects
// NOTE: ALL folder names should not be followed by a /

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

// store_object stores the given object in the given filename
// obj: is an object_server Object that is to be stored
// filename: is the name you want the object to be stored in
// return: bool indicated whether this save was successful
bool store_object(Object* obj, string filename);

// store_env stores the entire environment into disk memory
// env_ext: is the folder where environments are stored
// env_name: is the name to ssave this environment as
// objects: is the object_vector of all the objects that are going to be stored
// return: bool indicating whether this object was successfully saved
bool store_env(string env_ext, string env_name, vector<Object*> objects);

// revive_object loads the object from disk memory into an Object
// filename: is the name of the file to restore the object from
// ids_set_: is a list of the id's that have been set (so none are written over and they can be updated)
// object: is the Object that will be filled with the object's information
// object_count: How many objects have been set? This determines what this object will be
// return: bool indicating whether this Object was successfully revived
bool revive_object(string filename, vector<int>& ids_set_, Object** object, int object_count);

// restore_env loads an entire environment (list of objects) from memory
// env_ext: is the folder where the environments are stored
// env_name: is the name the environment has been stored under
// ids_set_: is the vector of ids that have been set (so none are over-written and this knowledge can be updated)
// objects: the vector of objects that will be populated with this environment
// object_count: current object count for setting ids
bool restore_env(string env_ext, string env_name, vector<int>& ids_set_, vector<Object*>& objects, int &object_count);

}
