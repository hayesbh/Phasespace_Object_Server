/**
 * File: load_env.cpp
 * Author: Dylan Visher
 * Date: 5/18/13
 * About: ROS client for the load_env service
 * Load an Environment from the disk
 */

#include <cstdlib>
#include <vector>
#include "ros/ros.h"
#include "core_object_server/load_env.h"

using std::vector;
using std::string;

int main(int argc, char **argv) {
  /*itintialize the ROS client*/
  ros::init(argc, argv, "load_env");
  /*Check for inproper number of arguments*/
  if (argc != 2) {
    ROS_WARN("usage: rosrun core_object_server load_env name");
    return 1;
  }
  /*Initialize the service client Node on the add_object service*/
  ros::NodeHandle n;
  ros::ServiceClient client =
    n.serviceClient<core_object_server::load_env>("load_env");
  core_object_server::load_env srv;
  srv.request.name = argv[1];
  /*If the call was successful*/
  ROS_INFO("Request Sent");
  if (client.call(srv)) {
    ROS_INFO("Environment Successfully Loaded");
  } else {
    ROS_WARN("Error in Loading");
  }
  return 0;
}
