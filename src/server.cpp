// File: server.cpp
// Author: Dylan Visher
// Date: 5/18/13
// About: ROS node for tracking objects in the PhaseSpace System

#include <sys/stat.h>
#include <sys/types.h>
// Include the ROS system
#include <ros/ros.h>
// rviz integration
#include <visualization_msgs/Marker.h>
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

// service files
#include "core_object_server/add_object.h"
#include "core_object_server/add_points.h"
#include "core_object_server/delete_object.h"
#include "core_object_server/box_filled.h"
#include "core_object_server/collides.h"
#include "core_object_server/save_object.h"
#include "core_object_server/load_object.h"
#include "core_object_server/add_manual.h"
#include "core_object_server/save_env.h"
#include "core_object_server/load_env.h"

// Set up ROS messages
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "core_object_server/ObjectInfo.h"
#include "core_object_server/ObjectDigest.h"

// Object functionality
#include "PhaseSpace/Object.h"
#include "PhaseSpace/PSObject.h"
#include "PhaseSpace/ManualObject.h"
#include "PhaseSpace/Point.h"

// OWL (PhaseSpace System API)
#include "owl/owl.h"

using std::vector;
using std::map;
using object_server::Object;
using object_server::Point;
using std::string;
using std::stringstream;
using object_server::FindPointById;
using object_server::ManualObject;
using object_server::PSObject;
using object_server::FindObjectByName;

// MARKER_COUNT is the maximum number of markers that will ever be used 
#define MARKER_COUNT 200
// SERVER_NAME is the IP Address for the PhaseSpace System
#define SERVER_NAME "192.168.2.123"
// This flag will set the OWL system up to be in not slave mode;
#define INIT_FLAGS 0
// OBJECT_FILE_EXT is the relative path of the saved object information
#define OBJECT_FILE_EXT "object_files/"
#define ENV_EXT "environments/"
// object_count is the number of objects set so far (deletions are not counted)
// It is used for assigning new identification numbers to objects
int object_count_ = 0;
// markers is the gloval array of OWLMarkers that are taken from the PhaseSpace System */
OWLMarker markers[MARKER_COUNT];
// ids_set_ is a vector of Marker/Point ID's that have been set
// This is used when finding unassigned id's to set */
vector<int> ids_set_;
int tracker_ = 0;

// TODO: POINTERS TO OBJECTS
// Make Pure Virtual Classes (Object and Object Type)
// Pass only pointers to Objects
// new PSObject *glove -> set the information -> cast as Object -> delete glove when done
vector<Object*> object_vector_;
// Camera Frame Information
// shift is a vector that moves the PhaseSpace Origin to the left hand side of the table
const float shift[3] = { 2.10094,0.637346,-1.24804 };
// rotate is a 3x3 matrix that defines the rotational transformation of from the Camera Origin
// to the local origin: It is in the form of {X, Y, Z}^-1
// Where X, Y, and Z are the desired axes in the coordinate system of the already shifted camera system
const float rotate[3][3] = {{-0.802454, -0.0236066, 0.599659},
                            {0.529394, 0.213418, 0.823575},
                            {-0.14712, 0.976347, -0.158438}};
/////////////////////////////
/// OWL PHASESPACE ERRROR ///
/////////////////////////////
void owl_print_error(const char *s, int n)
{
  if(n < 0) printf("%s: %d\n", s, n);
  else if(n == OWL_NO_ERROR) printf("%s: No Error\n", s);
  else if(n == OWL_INVALID_VALUE) printf("%s: Invalid Value\n", s);
  else if(n == OWL_INVALID_ENUM) printf("%s: Invalid Enum\n", s);
  else if(n == OWL_INVALID_OPERATION) printf("%s: Invalid Operation\n", s);
  else printf("%s: 0x%x\n", s, n);
}

////////////////////////////////////
//// CHANGING COORDINATE SYSTEM ////
////////////////////////////////////

// Transform Takes in an Owl Marker and changes its position to be relative to the new origin
//   mark: A PhaseSpace OWL API marker
// This shifts it using the globally defined shift vector
// This rotates it using the globally defined rotation matrix
void Transform(OWLMarker *mark) {
  float x = mark->x - shift[0];
  float y = mark->y - shift[1];
  float z = mark->z - shift[2];
  mark->x = rotate[0][0]*x + rotate[0][1]*y + rotate[0][2]*z;
  mark->y = rotate[1][0]*x + rotate[1][1]*y + rotate[1][2]*z;
  mark->z = rotate[2][0]*x + rotate[2][1]*y + rotate[2][2]*z;
  return;
}
// TransformMarkers transforms the position of n-many OWLMarkers in an Array]
// markers[]: An array of owl markers of length MARKER_COUNT]
// n: The number of markers to change
void TransformMarkers(OWLMarker markers[MARKER_COUNT], int n) {
  for (int i = 0; i < n; i++) {
    Transform(&markers[i]);
  } return;
}
//FindObject finds the location (iterator) of the object with the given id in the global object_vector_]
//id: the integer identification number for the object desired
//return: An iterator to the object in the global object_vector_
vector<Object*>::iterator FindObject(int id) {
  vector<Object*>::iterator iter;
  for (iter = object_vector_.begin(); iter != object_vector_.end(); ++iter)
    if ((*iter)->get_id() == id) return iter;
  return iter;
}

////////////////////////////////////////
//// ROS SERVICE AUXILLARY PROGRAMS ////
////////////////////////////////////////
//// get_unadded_points, get_points ////
////////////////////////////////////////

// get_unadded_points finds the valid points that have not been assigned yet
// return a vector of points that have not been assigned]
vector<Point> get_unadded_points() {
  int err;
  // Recieve Updated Information about the Markers
  int num_markers = owlGetMarkers(markers, MARKER_COUNT);
  // Transform these Markers to be reflective of the new Coordinate System
  TransformMarkers(markers, num_markers);
  // Catch errors in the OWL System: If there is an error shut the system down
  if ((err = owlGetError()) != OWL_NO_ERROR) {
    ROS_ERROR("get_unadded_points: owlGetError %d", err);
    owl_print_error("error", err);
    owlDone();
    exit(1);
  }
  
  // Make a vector of points that have not been added
  vector<Point> points;
  for (int i = 0; i < num_markers; i++) {
    // Only Add to Points if the valid marker has not been set and the data is good/reliable
    if (std::find(ids_set_.begin(), ids_set_.end(), markers[i].id) == ids_set_.end() && markers[i].cond > 0) {
      Point point;
      // Update the Point object to be reflective of the OWLMarker
      point.Update(markers[i]);
      points.push_back(point);
      // Record the id that has been set
      ids_set_.push_back(point.id);
    }
  }
  return points;
}

// get_points gathers all unassigned points for a given time
//            If there is an Object that needs a set amount of points (glove)
//            Assign the set of 7 points to be art of that object
//              [the PS LED Driver assignes 7 leds to each wired connection]
// time: The time in seconds the system will look for new points
// glove: A boolean that says whether to assign the whole 7 LED set to the object
// return a vector of unassigned points
vector<Point> get_points(int time, bool glove = false) {
  ROS_ASSERT(time >= 0);

  float rate = 10.0;  // frequency for looking for new unadded points (HZ)
  time *= rate;  // break time into blocks dependent on rate
  vector<Point> points;
  while (time > 0) {
    vector <Point> new_points = get_unadded_points();
    points.insert(points.end(), new_points.begin(), new_points.end());
    ros::Duration(1.0/rate).sleep();
    time--;
  }
  // If this is an object that takes up one whole connection
  // If at least one point has been gathered
  // And not all of them have been gathered
  // TODO: PERHAPS MAKE SURE THAT THIS DOESN'T ASSIGN LEDS FROM MORE THAN 1 WIRED CONNECTION
  if (glove && points.size() > 0 && points.size() < 7) {
    int mult = points[0].id / 7;
    vector<Point>::iterator iter;
    for (int i = 0; i < 7; i++) {
      // if the id was not found Initialize the point with that id
      // indicate that the associated id is now set
      if (FindPointById(i + mult * 7, points) == points.end()) {
        ids_set_.push_back(i + mult * 7);
        Point p;
        p.init();
        p.id = (i+ mult * 7);
        points.push_back(p);
      }
    }
  }
  return points;
}
bool store_object(Object* obj, string filename){
  vector<Point> points = obj->get_points();
  // Set up the JSON doc
  rapidjson::Document doc;
  rapidjson::Value json;
  json.SetObject();
  FILE *fp = fopen(filename.c_str(), "w");
  rapidjson::FileStream fs(fp);
  rapidjson::PrettyWriter<rapidjson::FileStream> writer(fs);
  writer.StartObject();
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
    X.SetDouble((double)my_center.x);
    center.PushBack(X, doc.GetAllocator());
    
    rapidjson::Value Y;
    X.SetDouble((double)my_center.y);
    center.PushBack(Y, doc.GetAllocator());

    rapidjson::Value Z;
    Z.SetDouble((double)my_center.z);
    center.PushBack(X, doc.GetAllocator());
    json.AddMember("center", center, doc.GetAllocator());

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
    // Set up the object_rigidity
    rapidjson::Value rigidity(obj->get_rigidity());
    json.AddMember("object_rigidity", rigidity, doc.GetAllocator());
    // Set up array of points
    rapidjson::Value point_id_array;
    point_id_array.SetArray();
    vector<Point>::iterator iter;
    for (iter = points.begin(); iter != points.end(); ++iter) {
      rapidjson::Value id;
      id.SetInt(iter->id);
      point_id_array.PushBack(id, doc.GetAllocator());
    }
    json.AddMember("object_points", point_id_array, doc.GetAllocator());
  }
  json.Accept(writer);
  writer.EndObject();
  fclose(fp);
  return true;
}
bool store_env(string env_name){
  string folder_name;
  folder_name.append(ENV_EXT).append(env_name);
  if(!mkdir(folder_name.c_str(), 777)) return false;
  if(!mkdir(folder_name.append("objects/").c_str(), 777)) return false;
  vector<Object*>::iterator iter;
  vector<string> names;
  for (iter = object_vector_.begin(); iter != object_vector_.end(); ++iter) {
    string filename = folder_name.append("objects/").append((*iter)->get_name());
    store_object(*iter, filename);
    names.push_back((*iter)->get_name());
  }
  rapidjson::Document doc;
  rapidjson::Value json;
  json.SetObject();
  FILE *fp = fopen(folder_name.append("objects").c_str(), "w");
  rapidjson::FileStream fs(fp);
  rapidjson::PrettyWriter<rapidjson::FileStream> writer(fs);
  writer.StartObject();
  rapidjson::Value objects;
  objects.SetArray();
  vector<string>::iterator it;
  for (it = names.begin(); it != names.end(); ++it) {
    rapidjson::Value name;
    name.SetString(it->c_str());
    objects.PushBack(name, doc.GetAllocator());
  }
  json.AddMember("objects", objects, doc.GetAllocator());
  json.Accept(writer);
  writer.EndObject();
  fclose(fp);
  return true;
}
bool revive_object(string filename){
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
    obj->init(object_count_, object_name);
    rapidjson::Value &json_center = p["center"];
    if (!json_center.IsArray() || json_center.Size() != 3) {
      ROS_ERROR("Manual Object Center is not a properly sized array");
      return false;
    } else {
      obj->SetCenter((float)json_center[(rapidjson::SizeType)0].GetDouble(), (float)json_center[1].GetDouble(), (float)json_center[2].GetDouble());
    }
    rapidjson::Value &dim = p["dimensions"];
    if (!dim.IsArray() || dim.Size() != 3) {
      ROS_ERROR("Manual Object dimensions is not a properly sized array");
      return false; 
    } else { 
      obj->SetDim((float)dim[(rapidjson::SizeType)0].GetDouble(), (float)dim[1].GetDouble(), (float)dim[2].GetDouble());
    }
    rapidjson::Value &angle = p["angle"];
    if (!angle.IsArray() || angle.Size() != 4) {
      ROS_ERROR("Manual Object angle is not a properly sized array");
      return false;
    } else {
      obj->SetAngle((float)angle[(rapidjson::SizeType)0].GetDouble(), (float)angle[1].GetDouble(), (float)angle[2].GetDouble(), (float)angle[3].GetDouble());
    }
    Object* casted = static_cast<Object*>(obj);
    object_vector_.push_back(casted);
    object_count_++;
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
        point.init();
        point.id = points[i].GetInt();
        if (std::find(ids_set_.begin(), ids_set_.end(), point.id) != ids_set_.end()) {
        ROS_ERROR("Point id (%i) has already been set.  Load Object (%s) failed", point.id, filename.c_str());
        fclose(fp);
        return false;
        }
      }
    }
    PSObject* obj = new PSObject;
    obj->init(object_count_, object_name, object_points, t, rigid);
    object_count_++;
    Object* casted = static_cast<Object*>(obj);
    object_vector_.push_back(casted);
  }
  fclose(fp);
  return true;
}

bool restore_env(string env_name) {
  string env_file = ENV_EXT;
  env_file.append(env_name);
  FILE *fp = fopen(env_file.c_str(), "r");
  rapidjson::FileStream fs(fp);
  rapidjson::Document p;
  if (!p.HasMember("objects")) {
    ROS_ERROR("Environment file improperly stored");
    return false;
  } else {
    rapidjson::Value &objects = p["objects"];
    for(rapidjson::SizeType i = 0; i < objects.Size(); ++i){
      string object_file = ENV_EXT;
      object_file.append("objects/").append(objects[i].GetString());
    }
  }
  return true; 
}

/////////////////////
//// ROS SERVICES////
//////////////////////////////////////////////////////////////////////////////// 
//// delete_object, add_object, add_points, collides, box_filled ////
////////////////////////////////////////////////////////////////////////////////

// delete_object deletes the Object with the Given ID from the list of Tracked Objects
// req: The ROS Service request for the core_object_server delete_object service
//      (int32) id: the id of the object to be deleted
// res: The ROS Service response fo the core_object_server delete_object service
// return: bool that indicates whether the service itself failed -> ROS_ERROR
bool delete_object(core_object_server::delete_object::Request &req,
                   core_object_server::delete_object::Response &res) {
  vector<Object*>::iterator target_iter = FindObject(req.id);
  if (target_iter == object_vector_.end()) {
    ROS_INFO("delete_object: target object(%i) does not exist", req.id);
    return true;
  }
  stringstream info;
  info << "Object (" << req.id << ") " "deleted\n";
  info << "Points: ";
  // Remove the Object's Associated Marker ID's from the list of set IDs
  Object* target = *target_iter;
  vector<Point> const &object_points = target->get_points();
  vector<Point>::const_iterator iter;
  for (iter = object_points.begin(); iter != object_points.end(); ++iter) {
    std::remove(ids_set_.begin(), ids_set_.end(), iter->id);
    info << iter->id << " ";
  }
  /*Remove the Object from the Tracked Objects List*/
  delete target;
  object_vector_.erase(target_iter, target_iter+1);
  /*Print out that this Object was successfully Removed*/
  info << "removed";
  ROS_INFO("%s", info.str().c_str());
  return true;
}

// add_points add points to a given object for a given time
// req: The ROS Service request for the core_object_server add_points service
//       (int32) id: The ID of the object to add points to
//       (int32) time: The time to be looking for new pointers
// res:  The ROS Service response for the core_object_server add_points service
//       (string) info: Information on how the program succeeded or failed
// return: bool that indeicates whether the service itself failed -> ROS_ERROR
bool add_points(core_object_server::add_points::Request &req,
                core_object_server::add_points::Response &res) {
  // Find target object for addition of points
  vector<Object*>::iterator target= FindObject(req.id);
  // If the object does not exist indicate a failed service
  if (target == object_vector_.end()) {
    ROS_INFO("add_points: no such target object exists\n");
    res.info = "No Such Object Exists";
    return true;
  }
  Object* object = *target;
  // Gather Additional Unset Points
  vector<Point> points = get_points(req.time);
  // If there were no points indicate teh the
  if (points.size() == 0) {
    ROS_WARN("add_points: No Additional Points Recieved");
    res.info.append("No Points Recieved");
    return true;
  }
  // Add the Points To this Object
  if (object->AddPoints(points)) {
    // Indicate a successfull points addition
    stringstream info;
    info << "Object: " << req.id << " has added the points: ";
    // Indicate which ID's were added to this object
    vector<Point>::const_iterator iter;
    for (iter = points.begin(); iter != points.end(); ++iter) {
      info << iter->id << " ";
    }
    ROS_INFO("%s", info.str().c_str());
    res.info = info.str();
    return true;
  } else {
    stringstream info;
    info << "Object: " << req.id << " is not a PhaseSpace Object";
    ROS_INFO("%s", info.str().c_str());
    res.info = info.str();
    vector<Point>::iterator iter;
    for (iter = points.begin(); iter != points.end(); ++iter) {
      std::remove(ids_set_.begin(), ids_set_.end(), iter->id);
    }
    return true;
  }
}

// add_object adds an object to be tracked
//            needs the axes to be visible when the alloted time is up
// req: The ROS Service request for the core_object_server add_object service
//       (string) name: User given name for the object
//       (int32)  time: User given time to be searching for new points
//       (string) type: Type of Object to be added
// res: The ROS Service response for the core_object_server add_object service
//       (string)  info: Information as to how the program succeeded or failed
// return: bool that indicates whether the service itself failed -> ROS_ERROR
bool add_object(core_object_server::add_object::Request &req,
                core_object_server::add_object::Response &res) {
  stringstream info;
  if (FindObjectByName(req.name, object_vector_) != object_vector_.end()) {
    info << "Name " << req.name << " is already in use";
    res.info = info.str();
  }
  // Find all the unassigned markers 
  vector<Point> points = get_points(req.time, (req.type == "glove"));
  // If no points were found indicate an unsuccessful addition
  if (points.size() == 0) {
    ROS_WARN("No Points Recieved");
    res.info.append("No Points Recieved");
    return false;
  }
  info << "Object " << req.name << " using points: ";
  vector<Point>::const_iterator iter;
  for (iter = points.begin(); iter != points.end(); ++iter) {
    info << iter->id << " ";
  }
  ROS_INFO("%s", info.str().c_str());
  // Initialize this New Object with the name given, new points, and the type
  PSObject* temp_object = new PSObject;
  temp_object->init(object_count_, req.name, points, req.type, req.rigid);
  // Add this object to the list of tracked objects
  Object* obj;
  obj = static_cast<Object*>(temp_object);
  object_vector_.push_back(obj);
  // pdate the universal object_count
  object_count_++;
  //Send the service caller the information associated with their call
  res.info = info.str();
  ROS_INFO("Object Added: %s\n", info.str().c_str());
  return true;
}
// add_manual adds a manual object to the environment
// req: The ROS Service requiest for the core_object_server add_manual service
//      (string) name: Unique user given name for object
//      (float[3]) center: Center for the new object
//      (float[3]) dim: Local dimensions for the object
//      (float[4]) rot: Rotation of Object (quaternion)
// res: The ROS Service response for the core_object_server add_manual service
//      (string) info: Information on how the service succeeded or failed
// return: bool that indicates whether the service succeeded
bool add_manual(core_object_server::add_manual::Request &req,
                       core_object_server::add_manual::Response &res) {
  stringstream info;
  if (FindObjectByName(req.name, object_vector_) != object_vector_.end()) {
    info << "Name " << req.name << " is already in use";
    res.info = info.str();
    return false;
  }
  ManualObject* obj = new ManualObject;
  obj->init(object_count_, req.name);
  info << object_count_ << ", " << req.name;
  obj->SetCenter(req.center[0], req.center[1], req.center[2]);
  obj->SetDim(req.dim[0], req.dim[1], req.dim[2]);
  obj->SetAngle(req.rot[0], req.rot[1], req.rot[2], req.rot[3]);
  object_count_++;
  Object* Obj = static_cast<Object*>(obj);
  object_vector_.push_back(Obj);
  res.info = info.str();
  ROS_INFO("Object Added: %s\n", info.str().c_str());
  return true;
}
// collides checks whether two objects with the given names collide
// req: The ROS Service request for the core_object_server collide service
//      name1: the string representing the first object in the questionable collision
//      name2: the string representing the second object in the collision
// res: The ROS service response for the core_object_server colled service
//      collides: bool that answers the question: The two objects are colliding?
// return: bool that indicates whether the service itself has failed -> ROS_ERROR
bool collides(core_object_server::collides::Request &req,
              core_object_server::collides::Response &res) {
  const vector<Object*>::iterator object1 = FindObjectByName(req.name1, object_vector_);
  const vector<Object*>::iterator object2 = FindObjectByName(req.name2, object_vector_);
  if (object1 == object_vector_.end() || object2 == object_vector_.end()) {
    res.collides = false;
    return true;
  } else res.collides = (*object1)->CollidesWith(*object2);
  return true;
}
// box filled checks whether a given box is filled
//     defined by a center(X,Y,Z) and side-length
// req: The ROS Service request for the core_object_server box_filled service
//      x: x coordinate for the center of the cube
//      y: y coordinate for the center of the cube
//      z: z coordinate for the center of the cube
//      width: the width of one side of the cube
// res: The ROS Service response for the core_object_server box_filled service
// return: bool that indicates whether the service itself failed -> ROS_ERROR
bool box_filled(core_object_server::box_filled::Request &req,
                core_object_server::box_filled::Response &res) {
  ManualObject* box = new ManualObject;
  box->init(-1, "cube");
  box->SetCenter( req.x, req.y, req.z );
  box->SetAngle(1, 0, 0, 0);
  box->SetDim(req.width, req.width, req.width);
  Object* obj = dynamic_cast<Object*>(box);
  vector<Object*>::iterator iter;
  res.filled = false;
  vector<string> objects_in;
  // If even one of the objects intersects the box then thje box is filled
  for (iter = object_vector_.begin(); iter != object_vector_.end(); ++iter) {
    if( (*iter)->CollidesWith(obj)) {
      res.filled = true;
      objects_in.push_back((*iter)->get_name());
    } 
  }
  res.objects = objects_in;
  return true;
}
// save_object stores the object into disk memory for later recall in a JSON file
// req: The ROS Service request for the core_object_server save_object service
//      name: the name of the object to be saved
//            it will be stored under this name for later recall
//            this name must be unique
// res: The ROS Service response for the core_object_server save_object service
//      info: information on the running of this service
// return: bool that indicates whether the service itself failed -> ROS_ERROR
bool save_object(core_object_server::save_object::Request &req,
                 core_object_server::save_object::Response &res) {
  string file = OBJECT_FILE_EXT;
  file += req.name;
  vector<Object*>::iterator obj = FindObjectByName(req.name, object_vector_);
  return store_object(*obj, file);
}

bool save_env(core_object_server::save_env::Request &req,
              core_object_server::save_env::Response &res) {
  return store_env(req.name);
}

bool load_env(core_object_server::load_env::Request &req,
              core_object_server::load_env::Response &res) {
  return restore_env(req.name);
}
// load_object loads the object from the JSON file where the objects are stored
// req: The ROS Service request for the core_object_server load_object service
//      name: the name of the object to be loaded from memory
// res: The ROS Service response for the core_object_server load_object service
//      info: information of the running of this service
// return: bool that indicates whether the service itself failed -> ROS_ERROR
bool load_object(core_object_server::load_object::Request &req,
                 core_object_server::load_object::Response &res) {
  string file = OBJECT_FILE_EXT;
  file += req.name;
  FILE *fp = fopen(file.c_str(), "r");
  rapidjson::FileStream fs(fp);
  rapidjson::Document p;
  if (p.ParseStream<0>(fs).HasParseError()) {
    ROS_ERROR("Parse error on filestream from load_object(%s).", file.c_str());
    fclose(fp);
    return false;
  }
  if (!p.IsObject()) {
    ROS_ERROR("Couldn't load object from file: %s. Not a JSON object.", file.c_str());
    fclose(fp);
    return false;
  }
  if (!p.HasMember("object_name")) {
    ROS_ERROR("Object in (%s) has no name field.", file.c_str());
    fclose(fp);
    return false;
  }
  if (!p.HasMember("object_type")) {
    ROS_ERROR("Object in (%s) has no type associated with it.", file.c_str());
    fclose(fp);
    return false;
  }
  if (!p.HasMember("object_rigidity")) {
    ROS_ERROR("Object in (%s) has an undefined rigidity.", file.c_str());
    fclose(fp);
    return false;
  }
  if (!p.HasMember("object_points")) {
    ROS_ERROR("Object in (%s) has no points associated with it.", file.c_str());
    fclose(fp);
    return false;
  }
  string object_name = p["object_name"].GetString();
  string t = p["type"].GetString();
  bool rigid = p["rigid"].GetBool();
  rapidjson::Value &points = p["points"];
  if (!points.IsArray()) {
    ROS_ERROR("Object points in (%s) is not an array.", file.c_str());
    fclose(fp);
    return false;
  } else {
    vector<Point> object_points;
    for(rapidjson::SizeType i = 0; i < points.Size(); ++i) {
      Point point;
      point.init();
      point.id = points[i].GetInt();
      if (std::find(ids_set_.begin(), ids_set_.end(), point.id) != ids_set_.end()) {
        ROS_ERROR("Point id (%i) has already been set.  Load Object (%s) failed", point.id, file.c_str());
        fclose(fp);
        return false;
      }
    }
    PSObject* obj = new PSObject;
    obj->init(object_count_, req.name, object_points, t, rigid);
    object_count_++;
    Object* casted = dynamic_cast<Object*>(obj);
    object_vector_.push_back(casted);
  }
  fclose(fp);
  return true;
}

// print_digest takes the ObjectDigest and Transforms it into a string
//    that can be displayed using ROS_INFO
// &digest: A ROS ObjectDigest message containing all the objects' information
string print_digest(core_object_server::ObjectDigest const &digest) {
  stringstream print;
  print << digest.time << "\n";
  vector<core_object_server::ObjectInfo>::const_iterator iter;
  for (iter = digest.objects.begin(); iter != digest.objects.end(); ++iter) {
    print << "(" << iter->id << ")";
    print << iter->name;
    print << "[" << iter->pos.x;
    print << "," << iter->pos.y;
    print << "," << iter->pos.z << "]";
    print << "[" << iter->rot.w << ",";
    print << iter->rot.x;
    print << "," << iter->rot.y << ",";
    print << iter->rot.z << "]" << "\n";
  } 
  return print.str();
}

// server: Integration into the OWL PhaseSpace System
// Outputs object information in the 
// ObjectDigest Message type
// On the objects ROS topic
int main(int argc, char **argv) {
  // Frequency for reading information from the PhaseSpace System
  // If frequency is increased make sure to check for lag
  // Increase in small increments
  int frequency = 10;
  /////////////////////////////////////
  /// Initialize PhaseSpace OWL API ///
  /////////////////////////////////////
  // This section follows the given PhaseSpace Example 1 for point tracking
  // Start up the Owl Server and make sure that it does not fail
  if (owlInit(SERVER_NAME, INIT_FLAGS) < 0) return 0;
  // Set up a point tracker and assign all markers to it
  owlTrackeri(tracker_, OWL_CREATE, OWL_POINT_TRACKER);
  for (int i = 0; i < MARKER_COUNT; i++)
    owlMarkeri(MARKER(tracker_, i), OWL_SET_LED, i);
  // Enable this tracker for streaming data
  owlTracker(tracker_, OWL_ENABLE);
  // Show if at any point there was an error in setting up this tracker
  if (!owlGetStatus()) {
    owl_print_error("error in point tracker setup", owlGetError());
    return 0;
  }
  // Set the streaming frequency tot the frequency defined earlier
  owlSetFloat(OWL_FREQUENCY, frequency);
  // Start streaming data from the PhaseSpace System
  owlSetInteger(OWL_STREAMING, OWL_ENABLE);
  // Set Scale of OWL System (millimeters -> meters)
  owlScale(0.001);
  ///////////////////////////
  /// Initialize ROS Node ///
  ///////////////////////////
  // Initialize the rosnode server which will output this data
  ros::init(argc, argv, "server");
  ros::NodeHandle n;
  // publish object information on core_object_server/objects channel
  ros::Publisher publisher =
    n.advertise<core_object_server::ObjectDigest>("objects", 0);
  // publish object information also to rviz
  ros::Publisher rviz_pub =
      n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  // Set rviz shape to be a rectangular prism
  uint32_t shape = visualization_msgs::Marker::CUBE;
  /*Provide ROS Services for adding objects, points, and deleting objects*/
  ros::ServiceServer add_obj =
      n.advertiseService("add_object", add_object);
  ros::ServiceServer add_pnts =
      n.advertiseService("add_points", add_points);
  ros::ServiceServer rm_obj =
      n.advertiseService("delete_object", delete_object);
  ros::ServiceServer intersect =
      n.advertiseService("box_filled", box_filled);
  ros::ServiceServer collide = 
      n.advertiseService("collides", collides);
  ros::ServiceServer save =
      n.advertiseService("save_object", save_object);
  ros::ServiceServer load =
      n.advertiseService("load_object", load_object);
  ros::ServiceServer man = 
      n.advertiseService("add_manual", add_manual);
  ros::ServiceServer save_environment =
      n.advertiseService("save_env", save_env);
  ros::ServiceServer load_environment =
      n.advertiseService("load_env", load_env);
  // The rate to loop throught the ROS calls
  // There is no need to do this at a higher rate than the PhaseSpace system
  ros::Rate loop_rate(frequency); 
  while (ros::ok()) {
    // Populate the markers with new position information from the PhaseSpace System
    int err;
    int n = owlGetMarkers(markers, MARKER_COUNT);
    // Transform these markers to be representative of the new coordinate system
    TransformMarkers(markers, n);
    // Put a timestamp on the messages
    ros::Time timestamp;
    timestamp = ros::Time::now();
    // If there was an error in getting the marker information indicate that
    if ((err = owlGetError()) != OWL_NO_ERROR) {
      owl_print_error("error", err);
    }
    // Initialize the message object for giving out Object Information
    core_object_server::ObjectDigest digest;
    digest.time = timestamp.sec;
    // If there are no Object's set then we are done
    if (object_vector_.size() == 0) {
      ROS_INFO("No Objects Set");
      publisher.publish(digest);
    // Otherwise opdate each object with the new marker information
    } else {
      vector<Object*>::iterator iter;
      for (iter = object_vector_.begin(); iter != object_vector_.end(); ++iter) {
        // Update Objects and Get Info
        (*iter)->Update(markers, n);
        // Get the positional data from the object
        Point position = (*iter)->get_center();
        // Get the rotational data from the object
        vector<float> rotation = (*iter)->get_rotation();
        // Make the message type to hold all of this information
        core_object_server::ObjectInfo info;
        info.id = (*iter)->get_id();
        info.name = (*iter)->get_name();
        // Set the message's position data
        geometry_msgs::Point loc;
        loc.x = position.x;
        loc.y = position.y;
        loc.z = position.z;
        info.pos = loc;
        // Set the message's rotation data
        geometry_msgs::Quaternion q;
        q.w = rotation[0];
        q.x = rotation[1];
        q.y = rotation[2];
        q.z = rotation[3];
        info.rot = q;
        // Set the message's dimensional data
        vector<float> dimensions = (*iter)->get_dimensions();
        info.dim.push_back(dimensions[0]);
        info.dim.push_back(dimensions[1]);
        info.dim.push_back(dimensions[2]);
        // Set information for pointer
        Point point;
        point = (*iter)->get_pointer();
        geometry_msgs::Point p;
        p.x = point.x;
        p.y = point.y;
        p.z = point.z;
        info.pointer = p;
        // Add this object to the ObjectDigest
        digest.objects.push_back(info);
        ////////////
        /// RVIZ ///
        ////////////
        visualization_msgs::Marker marker;
        // The name of the frame of reference
        marker.header.frame_id ="/tablesurface";
        // The time for the object
        marker.header.stamp = timestamp;
        // Set the NameSpoace and ID
        marker.ns = "PhaseSpace_Objects";
        marker.id = (*iter)->get_id();
        marker.type = shape;
        // Indicate that the object will need to be added
        marker.action = visualization_msgs::Marker::ADD;
        // Set the position for the Rviz marker
        marker.pose.position.x = position.x;
        marker.pose.position.y = position.y;
        marker.pose.position.z = position.z;
        // Set the rotation for the Rviz marker (quaternion)
        marker.pose.orientation.w = rotation[0];
        marker.pose.orientation.x = rotation[1];
        marker.pose.orientation.y = rotation[2];
        marker.pose.orientation.z = rotation[3];
        // Set the marker's scale
        marker.scale.x = dimensions[0];
        marker.scale.y = dimensions[1];
        marker.scale.z = dimensions[2];
        // Set the color of the object to be green and opaque
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;
        // Let the marker last just a little bit longer than the loop time
        // This will make sure that lag won't make the objects flicker
        marker.lifetime = ros::Duration(1.0/(frequency-4));
        rviz_pub.publish(marker);
      }
      // Publish this Object Data
      publisher.publish(digest);
      ROS_INFO("%s", print_digest(digest).c_str());
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  ros::spin();
  /*Clean up PhaseSpace system*/
  owlDone();
  return 0;
}
