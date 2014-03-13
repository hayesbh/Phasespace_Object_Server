/**
 * File: load_object.cpp
 * Author: Dylan Visher
 * Date: 5/18/13
 * About: ROS client for the load_object service
 * Load an Object from the disk
 */

#include <cstdlib>
#include <vector>
#include "ros/ros.h"
#include "Phasespace_Object_Server/load_object.h"

using std::vector;
using std::string;

int main(int argc, char **argv) {
  /*itintialize the ROS client*/
  ros::init(argc, argv, "load_object");
  /*Check for inproper number of arguments*/
  if (argc != 2) {
    ROS_WARN("usage: rosrun Phasespace_Object_Server load_object name");
    return 1;
  }
  /*Initialize the service client Node on the add_object service*/
  ros::NodeHandle n;
  ros::ServiceClient client =
    n.serviceClient<Phasespace_Object_Server::load_object>("load_object");
  Phasespace_Object_Server::load_object srv;
  srv.request.name = argv[1];
  /*If the call was successful*/
  ROS_INFO("Request Sent");
  if (client.call(srv)) {
    ROS_INFO("Object Successfully Loaded");
  } else {
    ROS_WARN("Error in Loading");
  }
  return 0;
}
