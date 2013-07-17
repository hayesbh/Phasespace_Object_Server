// File: server.cpp
// Author: Dylan Visher
// Date: 5/18/13
// About: ROS node for tracking objects in the PhaseSpace System

// Include the ROS system
#include <ros/ros.h>
// rviz integration
#include <visualization_msgs/Marker.h>
// C++ includes
#include <string>
#include <vector>
#include <algorithm>

// service files
#include "core_object_server/add_object.h"
#include "core_object_server/add_points.h"
#include "core_object_server/delete_object.h"
#include "core_object_server/collides_with.h"
#include "core_object_server/box_filled.h"

// Set up ROS messages
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "core_object_server/ObjectInfo.h"
#include "core_object_server/ObjectDigest.h"

// Object functionality
#include "PhaseSpace/ObjectClass.h"
#include "PhaseSpace/Point.h"

// OWL (PhaseSpace System API)
#include "owl/owl.h"

using std::vector;
using object_server::ObjectClass;
using object_server::Point;
using std::string;
using std::stringstream;
using object::FindById;

// MARKER_COUNT is the maximum number of markers that will ever be used 
#define MARKER_COUNT 200
// SERVER_NAME is the IP Address for the PhaseSpace System
#define SERVER_NAME "192.168.2.123"
// This flag will set the OWL system up to be in not slave mode;
#define INIT_FLAGS 0

// object_count is the number of objects set so far (deletions are not counted)
// It is used for assigning new identification numbers to objects
int object_count_ = 0;
// markers is the gloval array of OWLMarkers that are taken from the PhaseSpace System */
OWLMarker markers[MARKER_COUNT];
// ids_set_ is a vector of Marker/Point ID's that have been set
// This is used when finding unassigned id's to set */
vector<int> ids_set_;
int tracker_ = 0;
vector<ObjectClass> object_vector_;
// Camera Frame Information
// shift is a vector that moves the PhaseSpace Origin to the left hand side of the table
const float shift[3] = { 2.10094,0.637346,-1.24804 };
// rotate is a 3x3 matrix that defines the rotational transformation of from the Camera Origin
// to the local origin: It is in the form of {X, Y, Z}^-1
// Where X, Y, and Z are the desired axes in the coordinate system of the already shifted camera system
const float rotate[3][3] = {{-0.802454, -0.0236066, 0.599659},
                            {0.529394, 0.213418, 0.823575},
                            {-0.14712, 0.976347, -0.158438}};

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
vector<ObjectClass>::iterator FindObject(int id) {
  vector<ObjectClass>::iterator iter;
  for (iter = object_vector_.begin(); iter != object_vector_.end(); ++iter)
    if (iter->get_id() == id) return iter;
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
      if (points::FindById(i + mult * 7, points) == points.end()) {
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

/////////////////////
//// ROS SERVICES////
//////////////////////////////////////////////////////////////////////////////// 
//// delete_object, add_object, add_points, collision_detection, box_filled ////
////////////////////////////////////////////////////////////////////////////////

// delete_object deletes the Object with the Given ID from the list of Tracked Objects
// req: The ROS Service request for the core_object_server delete_object service
//      (int32) id: the id of the object to be deleted
// res: The ROS Service response fo the core_object_server delete_object service
//      (bool) success: Answers the question: Was the deletion successful?
// return: bool that indicates whether the service itself failed -> ROS_ERROR
bool delete_object(core_object_server::delete_object::Request &req,
                   core_object_server::delete_object::Response &res) {
  vector<ObjectClass>::iterator target_iter = FindObject(req.id);
  if (target_iter == object_vector_.end()) {
    ROS_INFO("delete_object: target object(%i) does not exist", req.id);
    res.success = false;
    return true;
  }
  stringstream info;
  info << "Object (" << req.id << ") " "deleted\n";
  info << "Points: ";
  // Remove the Object's Associated Marker ID's from the list of set IDs
  ObjectClass &target = *target_iter;
  vector<Point> const &object_points = target.get_points();
  vector<Point>::const_iterator iter;
  for (iter = object_points.begin(); iter != object_points.end(); ++iter) {
    std::remove(ids_set_.begin(), ids_set_.end(), iter->id);
    info << iter->id << " ";
  }
  /*Remove the Object from the Tracked Objects List*/
  object_vector_.erase(target_iter, target_iter+1);
  /*Print out that this Object was successfully Removed*/
  info << "removed";
  ROS_INFO("%s", info.str().c_str());
  res.success = true;
  return true;
}

// add_points add points to a given object for a given time
// req: The ROS Service request for the core_object_server add_points service
//       (int32) id: The ID of the object to add points to
//       (int32) time: The time to be looking for new pointers
// res:  The ROS Service response for the core_object_server add_points service
//       (bool) success: Answers the question : Were Points successfully added?
//       (string) info: Information on how the program succeeded or failed
// return: bool that indeicates whether the service itself failed -> ROS_ERROR
bool add_points(core_object_server::add_points::Request &req,
                core_object_server::add_points::Response &res) {
  // Find target object for addition of points
  vector<ObjectClass>::iterator target = FindObject(req.id);
  // If the object does not exist indicate a failed service
  if (target == object_vector_.end()) {
    res.success = false;
    ROS_INFO("add_points: no such target object exists\n");
    res.info = "No Such Object Exists";
    return true;
  }
  ObjectClass &object = *target;
  // Gather Additional Unset Points
  vector<Point> points = get_points(req.time);
  // If there were no points indicate teh the
  if (points.size() == 0) {
    ROS_WARN("add_points: No Additional Points Recieved");
    res.success = false;
    res.info.append("No Points Recieved");
    return true;
  }
  // Add the Points To this Object
  object.AddPoints(points);
  // Indicate a successfull points addition
  stringstream info;
  info << "Object: " << req.id << " has added the points: ";
  res.success = true;
  // Indicate which ID's were added to this object
  vector<Point>::const_iterator iter;
  for (iter = points.begin(); iter != points.end(); ++iter) {
    info << iter->id << " ";
  }
  ROS_INFO("%s", info.str().c_str());
  res.info = info.str();
  return true;
}

// add_object adds an object to be tracked
//            needs the axes to be visible when the alloted time is up
// req: The ROS Service request for the core_object_server add_object service
//       (string) name: User given name for the object
//       (int32)  time: User given time to be searching for new points
//       (string) type: Type of Object to be added
// res: The ROS Service response for the core_object_server add_object service
//       (bool) success: Answers the Question: Was an object successfully added
//       (string)  info: Information as to how the program succeeded or failed
// return: bool that indicates whether the service itself failed -> ROS_ERROR
bool add_object(core_object_server::add_object::Request &req,
                core_object_server::add_object::Response &res) {
  // Find all the unassigned markers 
  vector<Point> points = get_points(req.time, (req.type == "glove"));
  // If no points were found indicate an unsuccessful addition
  if (points.size() == 0) {
    ROS_WARN("No Points Recieved");
    res.success = false;
    res.info.append("No Points Recieved");
    return true;
  }
  // Add to the information which points are being added to the object
  stringstream info;
  info << "Object " << req.name << " using points: ";
  vector<Point>::const_iterator iter;
  for (iter = points.begin(); iter != points.end(); ++iter) {
    info << iter->id << " ";
  }
  ROS_INFO("%s", info.str().c_str());
  // Initialize this New Object with the name given, new points, and the type
  ObjectClass temp_object;
  temp_object.init(object_count_, req.name, points, req.type);
  /*Add this object to the list of tracked objects*/
  object_vector_.push_back(temp_object);
  /*Update the universal object_count*/
  object_count_++;
  /*Send the service caller the information associated with their call*/
  res.info = info.str();
  res.success = true;
  ROS_INFO("Object Added: %s\n", info.str().c_str());
  return true;
}
bool collision_detection(core_object_server::collides_with::Request &req,
                         core_object_server::collides_with::Response &res) {
  Point p;
  p.init(req.x, req.y, req.z);
  vector<ObjectClass>::iterator iter;
  res.collides = false;
  for (iter = object_vector_.begin(); iter != object_vector_.end(); ++iter)
    if (iter->CollidesWith(p)) { 
      res.collides = true;
      ROS_INFO("COLLISION DETECTED");
    }
  return true;
}
bool box_filled(core_object_server::box_filled::Request &req,
                core_object_server::box_filled::Response &res) {
  Point Center;
  Center.init(req.x, req.y, req.z);
  vector<ObjectClass>::iterator iter;
  res.filled = false;
  for (iter = object_vector_.begin(); iter != object_vector_.end(); ++iter)
    if( iter->IntersectsBox(Center, req.width)) res.filled = true;
  return true;
}
/* PRINTING OBJECT DATA */
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

/**server: Integration into the OWL PhaseSpace System
  *Keep track of the objects that are in the system*/
int main(int argc, char **argv) {
  int frequency = 10;
  /*** Initialize PhaseSpace OWL API ***/
  /*Integration into the OWL PhaseSpace API*/
  /*This section follows EXAMPLE 1 for point tracking*/
  /*Start up the Owl Server*/
  if (owlInit(SERVER_NAME, INIT_FLAGS) < 0) return 0;
  /*Assign all markers to one universal tracker*/
  owlTrackeri(tracker_, OWL_CREATE, OWL_POINT_TRACKER);
  for (int i = 0; i < MARKER_COUNT; i++)
    owlMarkeri(MARKER(tracker_, i), OWL_SET_LED, i);
  /*Enable this Tracker for Streaming*/
  owlTracker(tracker_, OWL_ENABLE);
  /*Throw Error if there is an Error in Tracker Set Up*/
  if (!owlGetStatus()) {
    owl_print_error("error in point tracker setup", owlGetError());
    return 0;
  }
  /*Set the Default Frequency to Maximum*/
  owlSetFloat(OWL_FREQUENCY, frequency);
  /*Start streaming from the PhaseSpace System*/
  owlSetInteger(OWL_STREAMING, OWL_ENABLE);
  /*set Scale of OWL System*/ 
  owlScale(0.001);
  // Rotate System
  // pos -2106.48, -636.975, 1248.58
  // {0.593436, 0., -0.803116, -0.0532773}
  // 0.283318, -0.805066, -0.0316099, 0.520193
  // q = 0.283227, 0., -0.95695, -0.0634824
  // const float pose[7] = { -2106.48, -636.975, 1248.58, 1, 0, 0, 0};
  // const float pose[7] = { 0, 0, 0, 0.283318, -0.805066, -0.0316099, 0.520193 };
  // const float pose[7] = { 0, 0, 0, 0.283227, 0, -0.95695, -0.0634824};
  // const float pose[7] = { 0, 0, 0, 0.283227, 0, -0.95695, -0.0634824};
  
  // owlLoadPose(pose);

  /*** Initialize ROS Node ***/
  /*Add in ROS functionality to make rosnode*/
  ros::init(argc, argv, "server");
  ros::NodeHandle n;
  /*publish object information on core_object_server/objects channel*/
  ros::Publisher publisher =
    n.advertise<core_object_server::ObjectDigest>("objects", 0);
  /*rviz publisher*/
  ros::Publisher rviz_pub =
      n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  /*Set rviz shape to be a cube*/
  uint32_t shape = visualization_msgs::Marker::CUBE;
  /*Provide ROS Services for adding objects, points, and deleting objects*/
  ros::ServiceServer add_obj =
      n.advertiseService("add_object", add_object);
  ros::ServiceServer add_pnts =
      n.advertiseService("add_points", add_points);
  ros::ServiceServer rm_obj =
      n.advertiseService("delete_object", delete_object);
  ros::ServiceServer collision =
      n.advertiseService("collides_with", collision_detection);
  ros::ServiceServer intersect =
      n.advertiseService("box_filled", box_filled);
  
  ros::Rate loop_rate(frequency); //30Hz operational rate

  while (ros::ok()) {
    /*Populate the markers with new Data from the PhaseSpace System*/
    int err;
    int n = owlGetMarkers(markers, MARKER_COUNT);
    TransformMarkers(markers, n);
    ros::Time timestamp;
    timestamp = ros::Time::now();
    if ((err = owlGetError()) != OWL_NO_ERROR) {
      owl_print_error("error", err);
    }
    /*If there are no Objects loop back*/
    if (n == 0) {
      ros::spinOnce();
      loop_rate.sleep();
      continue;
    }
    /*Make a vector of points that have new data*/
    // TODO: Do something with these new marker ids
    vector<int> available;
    for (int i = 0; i < n; i++) {
      available.push_back(markers[i].id);
    }

    core_object_server::ObjectDigest digest;
    digest.time = timestamp.sec;
    /*If there are no Object's set print that out*/
    if (object_vector_.size() == 0) {
      ROS_INFO("No Objects Set");
      publisher.publish(digest);
    /*Otherwise opdate each object with the new marker information*/
    } else {
      vector<ObjectClass>::iterator iter;
      for (iter = object_vector_.begin(); iter != object_vector_.end(); ++iter) {
        /*TODO Break this display routine into its own function*/
  
        /*Update Objects and Get Info*/
        iter->Update(markers, n);
        /*Get POS*/
        Point position = iter->get_pos();
        /*Get Rotation*/
        vector<float> rotation = iter->get_rotation();
        /*Make ObjectInfo*/
        core_object_server::ObjectInfo info;
        info.id = iter->get_id();
        info.name = iter->get_name();
        /*set position info*/
        geometry_msgs::Point loc;
        loc.x = position.x;
        loc.y = position.y;
        loc.z = position.z;
        info.pos = loc;
        /*set rotation info*/
        geometry_msgs::Quaternion q;
        q.w = rotation[0];
        q.x = rotation[1];
        q.y = rotation[2];
        q.z = rotation[3];
        info.rot = q;
        /*set dimensional information*/
        float dimensions[3] = {0, 0, 0};
        iter->get_dimensions(dimensions);
        info.dim.push_back(dimensions[0]);
        info.dim.push_back(dimensions[1]);
        info.dim.push_back(dimensions[2]);
        /*set info for pointer finger((0,0,0) if not pointer)*/
        Point point;
        point = iter->get_pointer();
        geometry_msgs::Point p;
        p.x = point.x;
        p.y = point.y;
        p.z = point.z;
        info.pointer = p;
        /*Add Object to ObjectDigest*/
        digest.objects.push_back(info);
        /*Publish to RVIZ*/
        visualization_msgs::Marker marker;
        /*frame of reference*/
        marker.header.frame_id ="/tablesurface";
        marker.header.stamp = timestamp;
        /*Set namespace and id*/
        marker.ns = "PhaseSpace_Objects";
        marker.id = iter->get_id();
        marker.type = shape;
        /*prepare to add object to rviz*/
        marker.action = visualization_msgs::Marker::ADD;
        /*set rviz object position*/
        marker.pose.position.x = position.x;
        marker.pose.position.y = position.y;
        marker.pose.position.z = position.z;
        /*set rviz object rotation*/
        marker.pose.orientation.x = rotation[0];
        marker.pose.orientation.y = rotation[1];
        marker.pose.orientation.z = rotation[2];
        marker.pose.orientation.w = rotation[3];
        /*set the marker scale*/
        marker.scale.x = dimensions[0];
        marker.scale.y = dimensions[1];
        marker.scale.z = dimensions[2];
        /*set color*/
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;
        marker.lifetime = ros::Duration(1.0/frequency);
        rviz_pub.publish(marker);
      }
    }
    /*Publish this data and continue looping*/
    publisher.publish(digest);
    ROS_INFO("%s", print_digest(digest).c_str());
    ros::spinOnce();
    loop_rate.sleep();
  }
  ros::spin();
  /*Clean up PhaseSpace system*/
  owlDone();
  return 0;
}
