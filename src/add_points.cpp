/**
 * File: add_points.cpp
 * Author: Dylan Visher
 * Date: 5/18/13
 * About: ROS client for the add_points server
 * Adds more points to the object
 */
#include <cstdlib>
#include "ros/ros.h"
#include "core_object_server/add_points.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "add_points");
  /*Check for improper argument*/
  if (argc != 3) {
    ROS_WARN("usage: rosrun test add_points name time");
    return 1;
  }
  /*initialize service client node on add_points service*/
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<core_object_server::add_points>("add_points");
  core_object_server::add_points srv;
  /*get the id and time*/
  srv.request.id = atol(argv[1]);
  /*default time is 5 seconds*/
  int t = 5;
  if (argc == 3)
    t = atol(argv[2]);
  srv.request.time = t;
  ROS_INFO("Request Sent");
  /*call the service and print out response*/
  if (client.call(srv)) {
    ROS_INFO("%s", srv.response.info.c_str());
  } else {
    ROS_WARN("%s", srv.response.info.c_str());
    return 1;
  }
  return 0;
}
