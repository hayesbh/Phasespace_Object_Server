/**
 * File: server.cpp
 * Author: Dylan Visher
 * Date: 5/18/13
 * About: ROS node for tracking objects in the PhaseSpace System
 */

/*Include the ROS system*/
#include <ros/ros.h>
/*rviz integration*/
#include <visualization_msgs/Marker.h>
/*C++ includes*/
#include <string>
#include <vector>
#include <algorithm>

/*service files*/
#include "core_object_server/add_object.h"
#include "core_object_server/add_points.h"
#include "core_object_server/delete_object.h"

/*Set up ROS messages*/
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "core_object_server/ObjectInfo.h"
#include "core_object_server/ObjectDigest.h"

/*Object functionality*/
#include "PhaseSpace/ObjectClass.h"
#include "PhaseSpace/Point.h"

/*OWL (PhaseSpace System API)*/
#include "owl/owl.h"

using std::vector;
using object_server::ObjectClass;
using object_server::Point;
using std::string;
using std::stringstream;
using object::FindById;

#define MARKER_COUNT 200
#define SERVER_NAME "192.168.2.123"
#define INIT_FLAGS 0

int object_count_ = 0;
OWLMarker markers[MARKER_COUNT];
vector<int> ids_set_;
int tracker_ = 0;
vector<ObjectClass> object_vector_;
const float shift[3] = { 2.10094,0.637346,-1.24804 };
const float rotate[3][3] = {{0.802454, 0.0236066, -0.599659},
                            {0.529394, 0.213418, 0.823575},
                            {-0.14712, 0.976347, -0.158438}};

void Transform(OWLMarker *mark) {
  float x = mark->x - shift[0];
  float y = mark->y - shift[1];
  float z = mark->z - shift[2];

  mark->x = -1*(rotate[0][0]*x + rotate[0][1]*y + rotate[0][2]*z);
  mark->y = rotate[1][0]*x + rotate[1][1]*y + rotate[1][2]*z;
  mark->z = rotate[2][0]*x + rotate[2][1]*y + rotate[2][2]*z;
  return;
}
void TransformMarkers(OWLMarker markers[MARKER_COUNT], int n) {
  for (int i = 0; i < n; i++) {
    Transform(&markers[i]);
  } return;
}

vector<ObjectClass>::iterator FindObject(int id) {
  vector<ObjectClass>::iterator iter;
  for (iter = object_vector_.begin(); iter != object_vector_.end(); ++iter)
    if (iter->get_id() == id) return iter;
  return iter;
}
/**
 * [get_unadded_points returns the points that are available but have not been set]
 * @return        [Points that have not been assigned]
 */
vector<Point> get_unadded_points() {
  int err;
  /*Recieve Updated Information about the Markers*/
  int num_markers = owlGetMarkers(markers, MARKER_COUNT);
  TransformMarkers(markers, num_markers);
  /*Catch errors in the OWL System*/
  if ((err = owlGetError()) != OWL_NO_ERROR) {
    ROS_ERROR("get_unadded_points: owlGetError %d", err);
    owl_print_error("error", err);
    owlDone();
    exit(1);
  }

  /*Make a vector of Point that have not been added*/
  vector<Point> points;
  if (num_markers == 0) {
    /*
     * If the OWL System has not been able to gather Data on Any Points
     * The OWL System is not working properly
     */
    //  ROS_WARN("get_unadded_points: no markers found\n");
    return points;
  }
  for (int i = 0; i < num_markers; i++) {
    /*Only Add to Points if the Marker has data and that data is good*/
    if (std::find(ids_set_.begin(), ids_set_.end(), markers[i].id) == ids_set_.end() && markers[i].cond > 0) {
      Point point;
      /*Push the information from the OWLMarker into the Point*/
      point.Update(markers[i]);
      points.push_back(point);
      /*Add this to the id's that have been set already*/
      ids_set_.push_back(point.id);
    }
  }
  return points;
}

/**
 * [get_points finds and returns a list of the unassigned but now available points for a set time]
 * @param  time [Time in Seconds the System will look for new points]
 * @return      [A vector of Points that are unassigned but available]
 */
vector<Point> get_points(int time) {
  ROS_ASSERT(time >= 0);

  int rate = 10;
  time *= rate; // break time into 100ms blocks
  vector<Point> points;
  while (time > 0) {
    vector <Point> new_points = get_unadded_points();
    points.insert(points.end(), new_points.begin(), new_points.end());
    /*TODO make this abstracted out dependent on rate*/
    ros::Duration(0.1).sleep();
    time--;
  }
  return points;
}

/**
 * [delete_object deletes an Object from the list of Tracked Objects]
 * @param  req [ROS delete_object service request]
 *             [id: the id of the object to be deleted]
 * @param  res [ROS delete_object service response]
 * @return     [success: To show success]
 */
bool delete_object(core_object_server::delete_object::Request &req,
                   core_object_server::delete_object::Response &res) {
  ROS_INFO("delete_object: Object Deletion: ID: %ld", req.id);
  vector<ObjectClass>::iterator target_iter = FindObject(req.id);
  if (target_iter == object_vector_.end()) {
    ROS_INFO("delete_object: target object does not exist");
    res.success = false;
    return true;
  }
  stringstream info;
  info << "Object (" << req.id << ") " "deleted\n";
  info << "Points: ";

  /*Remove the Object's Associated Marker ID's from the list of set IDs*/
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

/**
 * [add_points ROS Service to add points to a given object for a set time]
 * @param  req [ROS add_points service request]
 *             [id: ID of the Object to Add points to]
 *             [time: time in seconds to search for points]
 * @param  res [ROS add_points service response]
 *             [success: indicate whether this was successful]
 *             [info: send back information on the Points Added]
 * @return     [Show whether this service call has failed or succeeded]
 */
bool add_points(core_object_server::add_points::Request &req,
                core_object_server::add_points::Response &res) {
  ROS_INFO("Add Points to Object: %ld for %ld seconds", req.id, req.time);
  /*locate target object*/
  vector<ObjectClass>::iterator target = FindObject(req.id);
  if (target == object_vector_.end()) {
    ROS_INFO("add_points: no such target object exists\n");
    res.info = "No Such Object Exists";
    return true;
  }
  ObjectClass &object = *target;
  /*Gather points*/
  vector<Point> points = get_points(req.time);
  if (points.size() == 0) {
    ROS_WARN("add_points: No Additional Points Recieved");
    res.success = false;
    res.info.append("No Points Recieved");
    return true;
  }
  ROS_INFO("There are %ld additional points\n", points.size());
  vector<Point>::const_iterator i;
  /*Add the Points To this Object*/
  object.AddPoints(points);
  /*info string to eventually return*/
  stringstream info;
  info << "Object: " << req.id << " has added the points: ";
  res.success = true;
  /*create a new vector of ids to add*/
  vector<Point>::const_iterator iter;
  for (iter = points.begin(); iter != points.end(); ++iter) {
    info << iter->id << " ";
  }
  ROS_INFO("%s", info.str().c_str());
  res.info = info.str();
  return true;
}

/**
 * [add_object ROS service to track a new object
 *             Make sure that the Long axis is visible in the End]
 * @param  req [ROS add_object service request]
 *             [name: Name for the new object]
 *             [time: Time to look for markers]
 * @param  res [ROS add_object service response]
 *             [info: Details of name and points added]
 * @return     [Return whether the addition was successful] 
 */
bool add_object(core_object_server::add_object::Request &req,
                core_object_server::add_object::Response &res) {
  ROS_INFO("Adding Object: Name: %s, Given %i time",
                    req.name.c_str(), req.time);
  /*make a new object Object*/
  ObjectClass temp_object;
  /*Find the unassigned markers*/
  vector<Point> points = get_points(req.time);
  /*If no points were found reveal that adding this object was unsuccessful*/
  if (points.size() == 0) {
    ROS_WARN("No Points Recieved");
    res.success = false;
    res.info.append("No Points Recieved");
    return true;
  }
  /*Print out which points are being added to this new object*/
  stringstream info;
  info << "Object " << req.name << " using points: ";
  vector<Point>::const_iterator iter;
  for (iter = points.begin(); iter != points.end(); ++iter) {
    info << iter->id << " ";
  }
  ROS_INFO("%s", info.str().c_str());
  /*Initialize this New Object with the name given and the new points*/
  temp_object.init(object_count_, req.name, points);
  /*Add this object to the list of tracked objects*/
  object_vector_.push_back(temp_object);
  /*Update the universal object_count*/
  object_count_++;
  /*Send the service caller the information associated with their call*/
  res.info = info.str();
  ROS_INFO("Object Added: %s\n", info.str().c_str());
  return true;
}

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
        float dimensions[3] = {0, 0, 0};
        iter->get_dimensions(dimensions);
        info.dim.push_back(dimensions[0]);
        info.dim.push_back(dimensions[1]);
        info.dim.push_back(dimensions[2]);
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
