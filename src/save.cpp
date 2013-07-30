// File: save.cpp
// Author: Dylan Visher
// Date: 5/18/13
// About: Logic for Saving and storing objects

#include "save.h"

namespace object_server {
// Store Object obj in file filename
// obj: pointer to object to store
// filename: filename where it will be stored
// return: bool indicating whether this was successful
bool store_object(Object* obj, string filename) {
    vector<Point> points = obj->get_points();
  FILE *fp = fopen(filename.c_str(), "w");
  rapidjson::FileStream fs(fp);
  printf("rapid json filestream opened\n");
  rapidjson::PrettyWriter<rapidjson::FileStream> writer(fs);
  writer.StartObject();
  rapidjson::Document doc;
  rapidjson::Value json;
  json.SetObject();
  // Set up the object_name
  rapidjson::Value object_name;
  object_name.SetString(obj->get_name().c_str());
  json.AddMember("object_name", object_name, doc.GetAllocator());
  printf("json.AddMember(object_name) complete\n");
  // Set up the object_type
  rapidjson::Value object_type;
  object_type.SetString(obj->get_type().c_str());
  json.AddMember("object_type", object_type, doc.GetAllocator());
  printf("json.AddMember(object_type) complete\n");
  if (obj->get_type() == "manual"){
    printf("Object is manual object\n");
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
    center.PushBack(X, doc.GetAllocator());
    json.AddMember("center", center, doc.GetAllocator());
    printf("center set\n");
    vector<float> my_dim = obj->get_dimensions();
    rapidjson::Value dim;
    dim.SetArray();
    rapidjson::Value x_scale;
    x_scale.SetDouble((double)my_dim[0]);
    rapidjson::Value y_scale;
    x_scale.SetDouble((double)my_dim[1]);
    rapidjson::Value z_scale;
    z_scale.SetDouble((double)my_dim[2]);
    dim.PushBack(x_scale, doc.GetAllocator());
    dim.PushBack(y_scale, doc.GetAllocator());
    dim.PushBack(z_scale, doc.GetAllocator());
    json.AddMember("dimensions", dim, doc.GetAllocator());
    printf("dimensions set\n");
    vector<float> my_angle = obj->get_rotation();
    rapidjson::Value angle;
    angle.SetArray();
    printf("Angle SetArray()\n");
    rapidjson::Value w;
    w.SetDouble((double)my_angle[0]);
    rapidjson::Value x;
    x.SetDouble((double)my_angle[1]);
    rapidjson::Value y;
    y.SetDouble((double)my_angle[2]);
    rapidjson::Value z;
    z.SetDouble((double)my_angle[3]);
    printf("w, x, y, z SetDouble()\n");
    angle.PushBack(w, doc.GetAllocator());
    angle.PushBack(x, doc.GetAllocator());
    angle.PushBack(y, doc.GetAllocator());
    angle.PushBack(z, doc.GetAllocator());
    json.AddMember("angle", angle, doc.GetAllocator());
    printf("angle set\n");
  } else {
    printf("Object is PSObject\n");
    PSObject* obj = dynamic_cast<PSObject*>(obj);
    // Set up the object_rigidity
    rapidjson::Value rigidity(obj->get_rigidity());
    json.AddMember("rigid", rigidity, doc.GetAllocator());
    // Set up array of points
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
    vector<int> axis1 = obj->get_axis1_ids();
    rapidjson::Value axis1_ids;
    axis1_ids.SetArray();
    if(axis1.size() == 2) {
      axis1_ids.PushBack(axis1[0], doc.GetAllocator());
      axis1_ids.PushBack(axis1[1], doc.GetAllocator());
    }
    json.AddMember("axis1_ids", axis1_ids, doc.GetAllocator());
    // Store Original Axis1 vector
    Point o_axis1 = obj->get_original_axis1();
    rapidjson::Value original_axis1;
    original_axis1.SetArray();
    original_axis1.PushBack(o_axis1.x_, doc.GetAllocator());
    original_axis1.PushBack(o_axis1.y_, doc.GetAllocator());
    original_axis1.PushBack(o_axis1.z_, doc.GetAllocator());
    json.AddMember("original_axis1", original_axis1, doc.GetAllocator());
    // Store Axis2 Points
    vector<int> axis2 = obj->get_axis2_ids();
    rapidjson::Value axis2_ids;
    axis2_ids.SetArray();
    if(axis2.size() == 2) {
      axis2_ids.PushBack(axis2[0], doc.GetAllocator());
      axis2_ids.PushBack(axis2[0], doc.GetAllocator());
    }
    json.AddMember("axis2_ids", axis2_ids, doc.GetAllocator());
    // Store Original Axis1 vector
    Point o_axis2 = obj->get_original_axis2();
    rapidjson::Value original_axis2;
    original_axis2.SetArray();
    original_axis2.PushBack(o_axis2.x_, doc.GetAllocator());
    original_axis2.PushBack(o_axis2.y_, doc.GetAllocator());
    original_axis2.PushBack(o_axis2.z_, doc.GetAllocator());
    json.AddMember("original_axis2", original_axis2, doc.GetAllocator());
    // Store Dimensional Information
    vector<float> my_dim = obj->get_dimensions();
    rapidjson::Value dim;
    dim.SetArray();
    rapidjson::Value x_scale;
    x_scale.SetDouble((double)my_dim[0]);
    rapidjson::Value y_scale;
    x_scale.SetDouble((double)my_dim[1]);
    rapidjson::Value z_scale;
    z_scale.SetDouble((double)my_dim[2]);
    dim.PushBack(x_scale, doc.GetAllocator());
    dim.PushBack(y_scale, doc.GetAllocator());
    dim.PushBack(z_scale, doc.GetAllocator());
    json.AddMember("dimensions", dim, doc.GetAllocator());
  }
  printf("at the writing and closing\n");
  json.Accept(writer);
  printf("json.Accept(writer)\n");
  writer.EndObject();
  printf("End of Object\n");
  fclose(fp);
  return true;
}
// store_env stores the entire environment into disk memory
// env_foler is the folder where all the environments are stored
// env_name is the name to store this environment under
// return: bool indicating whether this object was successfully saved
bool store_env(string env_folder, string env_name, vector<Object*> objects) {
  string folder_name;
  folder_name.append(env_folder).append(env_name);
  if(!mkdir(folder_name.c_str(), 777)) return false;
  // make the objects folder
  if(!mkdir(folder_name.append("objects/").c_str(), 777)) return false;
  vector<Object*>::iterator iter;
  vector<string> names;
  // Make a file that stores each of the objects' information
  for (iter = objects.begin(); iter != objects.end(); ++iter) {
    string filename = folder_name.append("objects/").append((*iter)->get_name());
    store_object(*iter, filename);
    names.push_back((*iter)->get_name());
  }
  // Write a file that remembers all of the names of these objects
  rapidjson::Document doc;
  rapidjson::Value json;
  json.SetObject();
  FILE *fp = fopen(folder_name.append(env_name).c_str(), "w");
  rapidjson::FileStream fs(fp);
  rapidjson::PrettyWriter<rapidjson::FileStream> writer(fs);
  writer.StartObject();
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
  writer.EndObject();
  fclose(fp);
  return true;
}


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
  string t = p["type"].GetString();
  if(t == "manual") {
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
      obj->SetDim((float)dim[(rapidjson::SizeType)0].GetDouble(), (float)dim[1].GetDouble(), (float)dim[2].GetDouble());
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
    rapidjson::Value &o_ax2 = p["original_axis1"];
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
      dimensions.push_back((float)dim[1].GetDouble());
      dimensions.push_back((float)dim[2].GetDouble());
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
bool restore_env(string env_folder, string env_name, vector<int>& ids_set_, vector<Object*>& objects, int& object_count) {
  string env_file = env_folder;
  env_file.append(env_name);
  FILE *fp = fopen(env_file.c_str(), "r");
  rapidjson::FileStream fs(fp);
  rapidjson::Document p;
  if (!p.HasMember("objects")) {
    ROS_ERROR("Environment file improperly stored");
    fclose(fp);
    return false;
  } else {
    rapidjson::Value &json_objects = p["objects"];
    for(rapidjson::SizeType i = 0; i < json_objects.Size(); ++i){
      Object* obj;
      string object_file = env_folder;
      object_file.append("objects/").append(json_objects[i].GetString());
      if(!revive_object(object_file, ids_set_, &obj, object_count)) {
         ROS_WARN("Object File (%s) Improperly Loaded", object_file.c_str());
      } else {
        objects.push_back(obj);
        object_count++;
      }
    }
  }
  fclose(fp);
  return true;
}

}
