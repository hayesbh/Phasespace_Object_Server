// File: save.cpp
// Author: Dylan Visher
// Date: 5/18/13
// About: Logic for Saving and storing objects

#include "save.h"

namespace Phasespace_Object_Server {
// Store Object obj in file filename
// obj: pointer to object to store
// filename: filename where it will be stored
// return: bool indicating whether this was successful
bool store_object(Object* obj, string filename) {
  FILE *fp = fopen(filename.c_str(), "w");
  if (fp == NULL) return false;
  rapidjson::FileStream fs(fp);
  rapidjson::PrettyWriter<rapidjson::FileStream> writer(fs);
  rapidjson::Document doc;
  rapidjson::Value json;
  json.SetObject();
  // Set up the object_name
  rapidjson::Value object_name;
  object_name.SetString(obj->get_name().c_str());
  json.AddMember("object_name", object_name, doc.GetAllocator());
  // Set up the object_type
  rapidjson::Value object_type;
  object_type.SetString(obj->get_type().c_str());
  json.AddMember("object_type", object_type, doc.GetAllocator());
  if (obj->get_type() == "manual"){
    Point my_center = obj->get_center();
    rapidjson::Value center;
    center.SetArray();
    rapidjson::Value X;
    X.SetDouble((double)my_center.x_);
    center.PushBack(X, doc.GetAllocator());
    rapidjson::Value Y;
    Y.SetDouble((double)my_center.y_);
    center.PushBack(Y, doc.GetAllocator());
    rapidjson::Value Z;
    Z.SetDouble((double)my_center.z_);
    center.PushBack(Z, doc.GetAllocator());
    json.AddMember("center", center, doc.GetAllocator());
    vector<float> my_dim = obj->get_dimensions();
    rapidjson::Value dim;
    dim.SetArray();
    rapidjson::Value x_scale;
    x_scale.SetDouble((double)my_dim[0]);
    rapidjson::Value y_scale;
    y_scale.SetDouble((double)my_dim[1]);
    rapidjson::Value z_scale;
    z_scale.SetDouble((double)my_dim[2]);
    dim.PushBack(x_scale, doc.GetAllocator());
    dim.PushBack(y_scale, doc.GetAllocator());
    dim.PushBack(z_scale, doc.GetAllocator());
    json.AddMember("dimensions", dim, doc.GetAllocator());
    vector<float> my_angle = obj->get_rotation();
    rapidjson::Value angle;
    angle.SetArray();
    rapidjson::Value w;
    w.SetDouble((double)my_angle[0]);
    rapidjson::Value x;
    x.SetDouble((double)my_angle[1]);
    rapidjson::Value y;
    y.SetDouble((double)my_angle[2]);
    rapidjson::Value z;
    z.SetDouble((double)my_angle[3]);
    angle.PushBack(w, doc.GetAllocator());
    angle.PushBack(x, doc.GetAllocator());
    angle.PushBack(y, doc.GetAllocator());
    angle.PushBack(z, doc.GetAllocator());
    json.AddMember("angle", angle, doc.GetAllocator());
  } else {
    PSObject* ps_obj = new PSObject();
    ps_obj = dynamic_cast<PSObject*>(obj);
    
    // Set up the object_rigidity
    std::cout << "obj rigidity: " << ps_obj->get_rigidity() << std::endl;
    rapidjson::Value rigidity(ps_obj->get_rigidity());
    json.AddMember("rigid", rigidity, doc.GetAllocator());
    // Set up array of points
    vector<Point> points = obj->get_points();
    rapidjson::Value point_id_array;
    point_id_array.SetArray();
    vector<Point>::iterator iter;
    for (iter = points.begin(); iter != points.end(); ++iter) {
      rapidjson::Value id;
      id.SetInt(iter->id_);
      point_id_array.PushBack(id, doc.GetAllocator());
    }
    json.AddMember("points", point_id_array, doc.GetAllocator());
    // Store Axis1 Points
    vector<int> axis1 = ps_obj->get_axis1_ids();
    rapidjson::Value axis1_ids;
    axis1_ids.SetArray();
    if(axis1.size() == 2) {
      axis1_ids.PushBack(axis1[0], doc.GetAllocator());
      axis1_ids.PushBack(axis1[1], doc.GetAllocator());
    }
    json.AddMember("axis1_ids", axis1_ids, doc.GetAllocator());
    // Store Original Axis1 vector
    Point o_axis1 = ps_obj->get_original_axis1();
    rapidjson::Value original_axis1;
    original_axis1.SetArray();
    original_axis1.PushBack(o_axis1.x_, doc.GetAllocator());
    original_axis1.PushBack(o_axis1.y_, doc.GetAllocator());
    original_axis1.PushBack(o_axis1.z_, doc.GetAllocator());
    json.AddMember("original_axis1", original_axis1, doc.GetAllocator());
    // Store Axis2 Points
    vector<int> axis2 = ps_obj->get_axis2_ids();
    rapidjson::Value axis2_ids;
    axis2_ids.SetArray();
    if(axis2.size() == 2) {
      axis2_ids.PushBack(axis2[0], doc.GetAllocator());
      axis2_ids.PushBack(axis2[1], doc.GetAllocator());
    }
    json.AddMember("axis2_ids", axis2_ids, doc.GetAllocator());
    // Store Original Axis1 vector
    Point o_axis2 = ps_obj->get_original_axis2();
    rapidjson::Value original_axis2;
    original_axis2.SetArray();
    original_axis2.PushBack(o_axis2.x_, doc.GetAllocator());
    original_axis2.PushBack(o_axis2.y_, doc.GetAllocator());
    original_axis2.PushBack(o_axis2.z_, doc.GetAllocator());
    json.AddMember("original_axis2", original_axis2, doc.GetAllocator());
    // Store Dimensional Information
    vector<float> my_dim = ps_obj->get_dimensions();
    rapidjson::Value dim;
    dim.SetArray();
    rapidjson::Value x_scale;
    x_scale.SetDouble((double)my_dim[0]);
    rapidjson::Value y_scale;
    y_scale.SetDouble((double)my_dim[1]);
    rapidjson::Value z_scale;
    z_scale.SetDouble((double)my_dim[2]);
    dim.PushBack(x_scale, doc.GetAllocator());
    dim.PushBack(y_scale, doc.GetAllocator());
    dim.PushBack(z_scale, doc.GetAllocator());
    json.AddMember("dimensions", dim, doc.GetAllocator());
  }
  json.Accept(writer);
  fclose(fp);
  return true;
}

// store_env stores the entire environment into disk memory
// env_ext: is the folder where environments are stored
// env_name: is the name to ssave this environment as
// objects: is the object_vector of all the objects that are going to be stored
// return: bool indicating whether this object was successfully saved
bool store_env(string env_folder, string env_name, vector<Object*> objects) {
  if(mkdir(env_folder.c_str(), 0777) != 0 && !EEXIST) return false;
  string folder_name;
  folder_name.append(env_folder).append("/").append(env_name);
  if(mkdir(folder_name.c_str(), 0777) != 0 && !EEXIST) return false;
  // make the objects folder
  if(mkdir((folder_name + "/objects/").c_str(), 0777) != 0 && !EEXIST) return false;
  vector<Object*>::iterator iter;
  vector<string> names;
  // Make a file that stores each of the objects' information
  for (iter = objects.begin(); iter != objects.end(); ++iter) {
    string filename = folder_name + "/objects/" + (*iter)->get_name();
    if(!store_object(*iter, filename)) return false;
    names.push_back((*iter)->get_name());
  }
  // Write a file that remembers all of the names of these objects
  rapidjson::Document doc;
  rapidjson::Value json;
  json.SetObject();
  FILE *fp = fopen((folder_name + "/" + env_name).c_str(), "w");
  if (fp == NULL) return false;
  rapidjson::FileStream fs(fp);
  rapidjson::PrettyWriter<rapidjson::FileStream> writer(fs);
  rapidjson::Value json_objects;
  json_objects.SetArray();
  vector<string>::iterator it;
  for (it = names.begin(); it != names.end(); ++it) {
    rapidjson::Value name;
    name.SetString(it->c_str());
    json_objects.PushBack(name, doc.GetAllocator());
  }
  json.AddMember("objects", json_objects, doc.GetAllocator());
  json.Accept(writer);
  fclose(fp);
  return true;
}

// revive_object loads the object from disk memory into an Object
// filename: is the name of the file to restore the object from
// ids_set_: is a list of the id's that have been set (so none are written over and they can be updated)
// object: is the Object that will be filled with the object's information
// object_count: How many objects have been set? This determines what this object will be
// return: bool indicating whether this Object was successfully revived
bool revive_object(string filename, vector<int>& ids_set_, Object** object, int object_count) {
  FILE *fp = fopen(filename.c_str(), "r");
  rapidjson::FileStream fs(fp);
  rapidjson::Document p;
  if (p.ParseStream<0>(fs).HasParseError()) {
    ROS_ERROR("Parse error on filestream from load_object(%s).", filename.c_str());
    fclose(fp);
    return false;
  }
  if (!p.IsObject()) {
    ROS_ERROR("Couldn't load object from file: %s. Not a JSON object.", filename.c_str());
    fclose(fp);
    return false;
  }
  if (!p.HasMember("object_name")) {
    ROS_ERROR("Object in (%s) has no name field.", filename.c_str());
    fclose(fp);
    return false;
  }
  if (!p.HasMember("object_type")) {
    ROS_ERROR("Object in (%s) has no type associated with it.", filename.c_str());
    fclose(fp);
    return false;
  }
  string object_name = p["object_name"].GetString();
  string t = p["object_type"].GetString();
  if(t == "manual") {
    if (!p.HasMember("center")) {
      ROS_ERROR("Object in (%s) has no center field.", filename.c_str());
      fclose(fp);
      return false;
    }
    if (!p.HasMember("dimensions")) {
      ROS_ERROR("Object in (%s) has no dimensions field.", filename.c_str());
      fclose(fp);
      return false;
    }
    if (!p.HasMember("angle")) {
      ROS_ERROR("Object in (%s) has no angle field.", filename.c_str());
      fclose(fp);
      return false;
    }
     
    ManualObject* obj = new ManualObject;
    obj->Init(object_count, object_name);
    rapidjson::Value &json_center = p["center"];
    if (!json_center.IsArray() || json_center.Size() != 3) {
      ROS_ERROR("Manual Object Center is not a properly sized array");
      fclose(fp);
      return false;
    } else {
      obj->SetCenter((float)json_center[(rapidjson::SizeType)0].GetDouble(), (float)json_center[1].GetDouble(), (float)json_center[2].GetDouble());
    }
    rapidjson::Value &dim = p["dimensions"];
    if (!dim.IsArray() || dim.Size() != 3) {
      ROS_ERROR("Manual Object dimensions is not a properly sized array");
      fclose(fp);
      return false;
    } else {
      obj->SetDimensions((float)dim[(rapidjson::SizeType)0].GetDouble(), (float)dim[1].GetDouble(), (float)dim[2].GetDouble());
    }
    rapidjson::Value &angle = p["angle"];
    if (!angle.IsArray() || angle.Size() != 4) {
      ROS_ERROR("Manual Object angle is not a properly sized array");
      fclose(fp);
      return false;
    } else {
      obj->SetAngle((float)angle[(rapidjson::SizeType)0].GetDouble(), (float)angle[1].GetDouble(), (float)angle[2].GetDouble(), (float)angle[3].GetDouble());
    }
    Object* casted = static_cast<Object*>(obj);
    casted->UpdateFields();
    *object = casted;
  } else {
    if (!p.HasMember("rigid")) {
      ROS_ERROR("PSObject in (%s) has no rigid field.", filename.c_str());
      fclose(fp);
      return false;
    }
    if (!p.HasMember("points")) {
      ROS_ERROR("PSObject in (%s) has no points field.", filename.c_str());
      fclose(fp);
      return false;
    }
    if (!p.HasMember("axis1_ids")) {
      ROS_ERROR("PSObject in (%s) has no axis1_ids field.", filename.c_str());
      fclose(fp);
      return false;
    }
    if (!p.HasMember("axis2_ids")) {
      ROS_ERROR("PSObject in (%s) has no axis2_ids field.", filename.c_str());
      fclose(fp);
      return false;
    }
    if (!p.HasMember("original_axis1")) {
      ROS_ERROR("PSObject in (%s) has no original_axis1 field.", filename.c_str());
      fclose(fp);
      return false;
    }
    if (!p.HasMember("original_axis2")) {
      ROS_ERROR("PSObject in (%s) has no original_axis2 field.", filename.c_str());
      fclose(fp);
      return false;
    }
    if (!p.HasMember("dimensions")) {
      ROS_ERROR("Object in (%s) has no dimensions field.", filename.c_str());
      fclose(fp);
      return false;
    }
    bool rigid = p["rigid"].GetBool();
    rapidjson::Value &points = p["points"];
    vector<Point> object_points;
    if (!points.IsArray()) {
      ROS_ERROR("Object points in (%s) is not an array.", filename.c_str());
      fclose(fp);
      return false;
    } else {
      for(rapidjson::SizeType i = 0; i < points.Size(); ++i) {
        Point point;
        point.Init();
        point.id_ = points[i].GetInt();
        if (std::find(ids_set_.begin(), ids_set_.end(), point.id_) != ids_set_.end()) {
        ROS_ERROR("Point id (%i) has already been set.  Load Object (%s) failed", point.id_, filename.c_str());
        fclose(fp);
        return false;
        }
        object_points.push_back(point);
        ids_set_.push_back(point.id_);
      }
    }
    // recover Axis 1 ids
    rapidjson::Value &ids1 = p["axis1_ids"];
    vector<int> axis1_ids;
    if (!ids1.IsArray()) {
      ROS_ERROR("Axis was was not stored in a proper array");
      fclose(fp);
      return false;
    } else {
      if(ids1.Size() == 2) {
        axis1_ids.push_back(ids1[(rapidjson::SizeType)0].GetInt());
        axis1_ids.push_back(ids1[(rapidjson::SizeType)1].GetInt());
      }
    }
    // recover original Axis 1
    rapidjson::Value &o_ax1 = p["original_axis1"];
    if ( o_ax1.Size() != 3 ) {
      ROS_ERROR("Original Axis was not stored properly");
      fclose(fp);
      return false;
    }
    Point original_axis1;
    original_axis1.Init((float)o_ax1[(rapidjson::SizeType)0].GetDouble(), (float)o_ax1[1].GetDouble(), (float)o_ax1[2].GetDouble());
    // recover Axis 2 ids
    rapidjson::Value &ids2 = p["axis2_ids"];
    vector<int> axis2_ids;
    if (!ids2.IsArray()) {
      ROS_ERROR("Axis was was not stored in a proper array");
      fclose(fp);
      return false;
    } else {
      if(ids2.Size() == 2) {
        axis2_ids.push_back(ids2[(rapidjson::SizeType)0].GetInt());
        axis2_ids.push_back(ids2[(rapidjson::SizeType)1].GetInt());
      }
    }
    // recover original Axis 2
    rapidjson::Value &o_ax2 = p["original_axis2"];
    if ( o_ax2.Size() != 3 ) {
      ROS_ERROR("Original Axis was not stored properly");
      fclose(fp);
      return false;
    }
    Point original_axis2;
    original_axis2.Init((float)o_ax2[(rapidjson::SizeType)0].GetDouble(), (float)o_ax2[1].GetDouble(), (float)o_ax2[2].GetDouble());
    // get Dimensions
    rapidjson::Value &dim = p["dimensions"];
    vector<float> dimensions;
    if (!dim.IsArray() || dim.Size() != 3) {
      ROS_ERROR("PS Object dimensions is not a properly sized array");
      fclose(fp);
      return false;
    } else {
      dimensions.push_back((float)dim[(rapidjson::SizeType)0].GetDouble());
      dimensions.push_back((float)dim[(rapidjson::SizeType)1].GetDouble());
      dimensions.push_back((float)dim[(rapidjson::SizeType)2].GetDouble());
    }

    PSObject* obj = new PSObject;
    obj->Init(object_count, object_name, object_points, t, rigid);
    obj->SetRigid(rigid);
    obj->SetAxis1IDs(axis1_ids);
    obj->SetOriginalAxis1(original_axis1);
    obj->SetAxis2IDs(axis2_ids);
    obj->SetOriginalAxis2(original_axis2);
    obj->SetDimensions(dimensions[0], dimensions[1], dimensions[2]);
    Object* casted = static_cast<Object*>(obj);
    casted->UpdateFields();
    *object = casted;
  }
  fclose(fp);
  return true;
}
// restore_env loads an entire environment (list of objects) from memory
// env_ext: is the folder where the environments are stored
// env_name: is the name the environment has been stored under
// ids_set_: is the vector of ids that have been set (so none are over-written and this knowledge can be updated)
// objects: the vector of objects that will be populated with this environment
// object_count: current object count for setting ids
bool restore_env(string env_folder, string env_name, vector<int>& ids_set_, vector<Object*>& objects, int& object_count) {
  env_folder.append("/").append(env_name).append("/");
  string env_file = env_folder;
  env_file.append(env_name);
  FILE *fp = fopen(env_file.c_str(), "r");
  rapidjson::FileStream fs(fp);
  rapidjson::Document p;
  if (p.ParseStream<0>(fs).HasParseError()) {
    ROS_ERROR("Parse error on filestream from load_object(%s).", env_file.c_str());
    fclose(fp);
    return false;
  }

  vector<string> file_names;
  if (!p.HasMember("objects")) {
    ROS_ERROR("Environment file improperly stored");
    fclose(fp);
    return false;
  } else {
    rapidjson::Value &json_objects = p["objects"];
    for(rapidjson::SizeType i = 0; i < json_objects.Size(); ++i){
      file_names.push_back(json_objects[i].GetString());
    }
  }
  fclose(fp);
  vector<string>::iterator iter;
  for (iter = file_names.begin(); iter != file_names.end(); ++iter) {
    Object* obj;
    string object_file = env_folder;
    object_file.append("/objects/").append(*iter);
    if(!revive_object(object_file, ids_set_, &obj, object_count)) {
       ROS_WARN("Object File (%s) Improperly Loaded", object_file.c_str());
    } else {
      objects.push_back(obj);
      object_count++;
    }
  }
  return true;
}

}
