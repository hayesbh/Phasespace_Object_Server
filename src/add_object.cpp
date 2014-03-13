/**
 * File: add_object.cpp
 * Author: Dylan Visher
 * Date: 5/18/13
 * About: ROS client for the add_object server
 * Adds this obejct to the tracked objects
 */

#include <cstdlib>
#include "ros/ros.h"
#include "Phasespace_Object_Server/add_object.h"

int main(int argc, char **argv) {
  /*itintialize the ROS client*/
  ros::init(argc, argv, "add_object");
  /*Check for inproper number of arguments*/
  if (argc != 5) {
    ROS_WARN("usage: rosrun test add_object name time type rigid");
    return 1;
  }
  /*Initialize the service client Node on the add_object service*/
  ros::NodeHandle n;
  ros::ServiceClient client =
    n.serviceClient<Phasespace_Object_Server::add_object>("add_object");
  Phasespace_Object_Server::add_object srv;
  srv.request.name = argv[1];
  /*Otherwise set it to the time the user gives*/
  int t = atoi(argv[2]);
  srv.request.time = t;
  srv.request.type = argv[3];
  if(atoi(argv[4]))
    srv.request.rigid = 1;
  else srv.request.rigid = 0;
  /*If the call was successful*/
  ROS_INFO("Request Sent");
  if (client.call(srv)) {
    /*display the returned information*/
    ROS_INFO("%s", srv.response.info.c_str());
  } else {
    ROS_WARN("%s", srv.response.info.c_str());
    return 1;
  }
  return 0;
}
