/**
 * File: add_object.cpp
 * Author: Dylan Visher
 * Date: 5/18/13
 * About: ROS client for the add_object server
 * Adds this obejct to the tracked objects
 */

#include <cstdlib>
#include "ros/ros.h"
#include "core_object_server/add_object.h"

int main(int argc, char **argv) {
  /*itintialize the ROS client*/
  ros::init(argc, argv, "add_object");
  /*Check for inproper number of arguments*/
  if (argc != 2 && argc != 3) {
    ROS_WARN("usage: rosrun test add_object name [time]");
    return 1;
  }
  /*Initialize the service client Node on the add_object service*/
  ros::NodeHandle n;
  ros::ServiceClient client =
    n.serviceClient<core_object_server::add_object>("add_object");
  core_object_server::add_object srv;
  srv.request.name = argv[1];
  /*default time is one second*/
  int time = 5;
  /*Otherwise set it to the time the user gives*/
  if (argc == 3) time = atol(argv[2]);
  srv.request.time = time;
  /*If the call was successful*/
  if (client.call(srv)) {
    /*display the returned information*/
    ROS_INFO("Request Sent");
    if (!(static_cast<int>(srv.response.success)))
      ROS_WARN("%s", srv.response.info.c_str());
    else
      ROS_INFO("%s", srv.response.info.c_str());
  } else {
    ROS_ERROR("Failed to call service");
    return 1;
  }
  return 0;
}
