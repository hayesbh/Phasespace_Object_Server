/**
 * File: save_env.cpp
 * Author: Dylan Visher
 * Date: 5/18/13
 * About: ROS client for the save_env service
 * Save an environment (many objects) to disk
 */

#include <cstdlib>
#include <vector>
#include "ros/ros.h"
#include "Phasespace_Object_Server/save_env.h"

using std::vector;
using std::string;

int main(int argc, char **argv) {
  /*itintialize the ROS client*/
  ros::init(argc, argv, "save_env");
  /*Check for inproper number of arguments*/
  if (argc != 2) {
    ROS_WARN("usage: rosrun Phasespace_Object_Serversave_env name");
    return 1;
  }
  /*Initialize the service client Node on the add_object service*/
  ros::NodeHandle n;
  ros::ServiceClient client =
    n.serviceClient<Phasespace_Object_Server::save_env>("save_env");
  Phasespace_Object_Server::save_env srv;
  srv.request.name = argv[1];
  /*If the call was successful*/
  ROS_INFO("Request Sent");
  if (client.call(srv)) {
    ROS_INFO("Environment Successfully Saved");
  } else {
    ROS_WARN("Error in Saving");
  }
  return 0;
}
