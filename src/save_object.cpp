/**
 * File: save_object.cpp
 * Author: Dylan Visher
 * Date: 5/18/13
 * About: ROS client for the save_object service
 * Save an Object to Disk
 */

#include <cstdlib>
#include <vector>
#include "ros/ros.h"
#include "core_object_server/save_object.h"

using std::vector;
using std::string;

int main(int argc, char **argv) {
  /*itintialize the ROS client*/
  ros::init(argc, argv, "save_object");
  /*Check for inproper number of arguments*/
  if (argc != 2) {
    ROS_WARN("usage: rosrun core_object_server save_object name");
    return 1;
  }
  /*Initialize the service client Node on the add_object service*/
  ros::NodeHandle n;
  ros::ServiceClient client =
    n.serviceClient<core_object_server::save_object>("save_object");
  core_object_server::save_object srv;
  srv.request.name = argv[1];
  /*If the call was successful*/
  ROS_INFO("Request Sent");
  if (client.call(srv)) {
    ROS_INFO("Object Successfully Saved");
  } else {
    ROS_WARN("Error in Saving");
  }
  return 0;
}
