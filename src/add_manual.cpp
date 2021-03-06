/**
 * File: add_manual.cpp
 * Author: Dylan Visher
 * Date: 5/18/13
 * About: ROS client for the add_object server
 * Adds this obejct to the tracked objects
 */

#include <cstdlib>
#include <vector>
#include "ros/ros.h"
#include "Phasespace_Object_Server/add_manual.h"

using std::vector;

int main(int argc, char **argv) {
  /*itintialize the ROS client*/
  ros::init(argc, argv, "add_manual");
  /*Check for inproper number of arguments*/
  if (argc != 12) {
    ROS_WARN("usage: rosrun Phasespace_Object_Server add_manual name x y z x_scale y_scale z_scale w x y z");
    return 1;
  }
  /*Initialize the service client Node on the add_object service*/
  ros::NodeHandle n;
  ros::ServiceClient client =
    n.serviceClient<Phasespace_Object_Server::add_manual>("add_manual");
  Phasespace_Object_Server::add_manual srv;
  srv.request.name = argv[1];
  for(int i = 2; i < 5; ++i) {
     srv.request.center.push_back(atof(argv[i]));
  }
  vector<float> dim;
  for(int i = 5; i < 8; ++i) {
     srv.request.dim.push_back(atof(argv[i]));
  }
  vector<float> rot;
  for(int i = 8; i < 12; ++i) {
     srv.request.rot.push_back(atof(argv[i]));
  }
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
