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

/*service files*/
#include "core_object_server/add_object.h"
#include "core_object_server/add_points.h"
#include "core_object_server/delete_object.h"

/*Set up ROS messages*/
#include "std_msgs/String.h"
#include "core_object_server/Point.h"
#include "core_object_server/Quaternion.h"
#include "core_object_server/ObjectInfo.h"
#include "core_object_server/ObjectDigest.h"

/*Object functionality*/
#include "PhaseSpace/ObjectClass.h"

/*OWL (PhaseSpace System API)*/
#include "owl/owl.h"

#define MARKER_COUNT 200
#define SERVER_NAME "192.168.2.123"
#define INIT_FLAGS 0

int object_count = 0;
OWLMarker markers[MARKER_COUNT];
vector<int> ids_set;
int tracker = 0;
vector<ObjectClass> objects_vector;

/**
 * [find finds a given int (id) in a vector (v)]
 * @param  id [int to find in vector]
 * @param  v  [vector to look in]
 * @return    [if success index if failiure -1]
 */
int find(int id, vector<int> v) {
  for (int i = 0; i < v.size(); i++) {
    if (id == v[i]) return i;
  }
  return -1;
}

/**
 * [get_unadded_points returns the points that are availible but have not been set]
 * @return        [Points that have not been assigned]
 */
vector<Point> get_unadded_points() {
  int err;
  /*Recieve Updated Information about the Markers*/
  int n = owlGetMarkers(markers, MARKER_COUNT);
  /*Catch errors in the OWL System*/
  if ((err = owlGetError()) != OWL_NO_ERROR) {
    owl_print_error("error", err);
    owlDone();
    exit(1);
  }
  /**If the OWL System has not been able to gather Data on Any Points
   * The OWL System is not working properly
   */
  /*Make a vector of Point that have not been added*/
  vector<Point> points;
  if (n == 0) {
    return points;
  }
  for (int i = 0; i < n; i++) {
    /*Only Add to Points if the Marker has data and that data is good*/
    if ((find(markers[i].id, ids_set) == -1) && (markers[i].cond > 0)) {
      Point point;
      /*Push the information from the OWLMarker into the Point*/
      point.Update(markers[i]);
      points.push_back(point);
      /*Add this to the id's that have been set already*/
       ids_set.push_back(point.id);
    }
  }
  return points;
}
/**
 * [get_points finds and returns a list of the unassigned but now availible points for a set time]
 * @param  time [Time in Seconds the System will look for new points]
 * @return      [A vector of Points that are unassigned but availible]
 */
vector<Point> get_points(int time) {
  printf("getting points\n");
  int rate = 30;
  time *= rate;
  vector<Point> points;
  ros::Rate r(rate);
  while (time != 0) {
    vector <Point> new_points = get_unadded_points();
    points.insert(points.end(), new_points.begin(), new_points.end());
    r.sleep();
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
  ROS_INFO("Object Deletion: ID: %ld", req.id);
  /** Find the index of this object
    * Return to caller if this object does not exist
    */
  int object_index = find(req.id, ids_set);
  if (object_index == -1) {
    res.success = false;
    return true;
  }
  ostringstream info;
  info << ("Object (%ld) ", req.id) << "deleted\n";
  info << "Points: ";

  vector<Point> object_points = objects_vector[object_index].get_points();
  printf("object points grabbed\n");
  /*Remove the Object from the Tracked Objects List*/
  objects_vector.erase(objects_vector.begin()+object_index,
                       objects_vector.begin()+object_index+1);
  printf("obeject erased\n");
  /*Remove the Object's Associated Marker ID's from the list of set IDs*/
  for (int i = 0; i < object_points.size(); i++) {
    int index = find(object_points[i].id, ids_set);
    ids_set.erase(ids_set.begin()+index, ids_set.begin()+index+1);
    printf("id: %i erased", object_points[i].id);
    info << object_points[i].id << " ";
  }
  /*Print out that this Object was successfully Removed*/
  info << "removed";
  ROS_INFO("%s", info.str().c_str());
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
  vector<Point> points = get_points(req.time);
  if (points.size() == 0) {
    ROS_WARN("No Additional Points Recieved");
    res.success = false;
    res.info.append("No Points Recieved");
    return true;
  }
  /*find out which object we are looking at*/
  int object_index = find(req.id, ids_set);
  /*Add the Points To this Object*/
  objects_vector[object_index].AddPoints(points);
  /*info string to eventually return*/
  ostringstream info;
  info << "Object: " << req.id << " has added the points: ";
  res.success = true;
  /*create a new vector of ids to add*/
  vector<int> ids;
  for (int i = 0; i < points.size(); i++) {
    info << points[i].id << " ";
  }
  ROS_INFO("%s", info.str().c_str());
  res.info = info.str();
  return true;
}
/**
 * [add_object ROS service to track a new object]
 * @param  req [ROS add_object service request]
 *             [name: Name for the new object]
 *             [time: Time to look for markers]
 * @param  res [ROS add_object service response]
 *             [info: Details of name and points added]
 * @return     [Return whether the addition was successful] 
 */
bool add_object(core_object_server::add_object::Request &req,
                core_object_server::add_object::Response &res) {
  ROS_INFO("Adding Object: Name: %s, Given %i time", req.name.c_str(), req.time);
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
  ostringstream info;
  info << "Object " << req.name << " using points: ";
  vector<int> ids;
  for (int i = 0; i < points.size(); i++) {
    info << points[i].id << " ";
  }
  ROS_INFO("%s", info.str().c_str());
  /*Initialize this New Object with the name given and the new points*/
  temp_object.init(object_count, req.name, points);
  /*Add this object to the list of tracked objects*/
  objects_vector.push_back(temp_object);
  /*Update the universal object_count*/
  object_count++;
  /*Send the service caller the information associated with their call*/
  res.info = info.str();
  return true;
}
string print_digest(core_object_server::ObjectDigest digest) {
  stringstream print;
  print << digest.time << "\n";
  for (int i = 0; i < digest.objects.size(); i++) {
    print << "(" << digest.objects[i].id << ")";
    print << digest.objects[i].name;
    print << "[" << digest.objects[i].pos.x;
    print << "," << digest.objects[i].pos.y;
    print << "," << digest.objects[i].pos.z << "]";
    print << "[" << digest.objects[i].rot.w << ",";
    print << digest.objects[i].rot.x;
    print << "," << digest.objects[i].rot.y << ",";
    print << digest.objects[i].rot.z << "]" << "\n";
  } return print.str();
}
/**server: Integration into the OWL PhaseSpace System
  *Keep track of the objects that are in the system*/
int main(int argc, char **argv) {
  /*Integration into the OWL PhaseSpace API*/
  /*This section follows EXAMPLE 1 for point tracking*/
  /*Start up the Owl Server*/
  if (owlInit(SERVER_NAME, INIT_FLAGS) < 0) return 0;
  /*Assign all markers to one universal tracker*/
  owlTrackeri(tracker, OWL_CREATE, OWL_POINT_TRACKER);
  for (int i = 0; i < MARKER_COUNT; i++)
    owlMarkeri(MARKER(tracker, i), OWL_SET_LED, i);
  /*Enable this Tracker for Streaming*/
  owlTracker(tracker, OWL_ENABLE);
  /*Throw Error if there is an Error in Tracker Set Up*/
  if (!owlGetStatus()) {
    owl_print_error("error in point tracker setup", owlGetError());
    return 0;
  }
  /*Set the Default Frequency to Maximum*/
  owlSetFloat(OWL_FREQUENCY, 20);
  /*Start streaming from the PhaseSpace System*/
  owlSetInteger(OWL_STREAMING, OWL_ENABLE);
  /*Add in ROS functionality to make rosnode*/
  ros::init(argc, argv, "server");
  ros::NodeHandle n;
  /*publish object information on info channel*/
  ros::Publisher publisher = n.advertise<core_object_server::ObjectDigest>("info", 1000);
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
  ros::Rate loop_rate(20);
  while (ros::ok()) {
    /*Populate the markers with new Data from the PhaseSpace System*/
    int err;
    int n = owlGetMarkers(markers, MARKER_COUNT);
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
    vector<int> availible;
    for (int i = 0; i < n; i++) {
      availible.push_back(markers[i].id);
    }
    core_object_server::ObjectDigest digest;
    digest.time = timestamp.sec;
    /*If there are no Object's set print that out*/
    if (objects_vector.size() == 0) {
      ROS_INFO("No Objects Set");
      publisher.publish(digest);
    /*Otherwise opdate each object with the new marker information*/
    } else {
      for (int i = 0; i < objects_vector.size(); i++) {
        /*Update Objects and Get Info*/
        objects_vector[i].Update(markers, n);
        /*Get POS*/
        Point position = objects_vector[i].get_pos();
        /*Get Rotation*/
        vector<float> rotation = objects_vector[i].get_rotation();
        /*Make ObjectInfo*/
        core_object_server::ObjectInfo info;
        info.id = objects_vector[i].get_id();
        info.name = objects_vector[i].get_name();
        /*set position info*/
        core_object_server::Point loc;
        loc.x = position.x;
        loc.y = position.y;
        loc.z = position.z;
        info.pos = loc;
        /*set rotation info*/
        core_object_server::Quaternion q;
        q.w = rotation[0];
        q.x = rotation[1];
        q.y = rotation[2];
        q.z = rotation[3];
        info.rot = q;
        /*Add Object to ObjectDigest*/
        digest.objects.push_back(info);
        /*Publish to RVIZ*/
        visualization_msgs::Marker marker;
        /*frame of reference*/
        marker.header.frame_id ="/base_link";
        marker.header.stamp = timestamp;
        /*Set namespace and id*/
        marker.ns = "PhaseSpace_Objects";
        marker.id = objects_vector[i].get_id();
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
        marker.scale.x = 100;
        marker.scale.y = 100;
        marker.scale.z = 100;
        /*set color*/
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;
        marker.lifetime = ros::Duration();
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



