/**
 * File: delete_object.cpp
 * Author: Dylan Visher
 * Date: 5/18/13
 * About: ROS client for the delete_object server
 * Delete object from the tracked
 */
#include <cstdlib>
#include "ros/ros.h"
#include "Phasespace_Object_Server/delete_object.h"

int main(int argc, char **argv) {
  /*initialize ROS*/
  ros::init(argc, argv, "add");
  /*check for proper number of arguments*/
  if (argc != 3) {
    ROS_WARN("usage: delete_object <id|name> <value>");
    return 1;
  }
  /*create the service client node on the delete_object service*/
  ros::NodeHandle n;
  ros::ServiceClient client =
    n.serviceClient<Phasespace_Object_Server::delete_object>("delete_object");
  Phasespace_Object_Server::delete_object srv;
  
  if (std::string(argv[1]).compare("id") == 0) {
    srv.request.id = atol(argv[2]);    
  } else {
    srv.request.name = argv[2];
  }
  
  ROS_INFO("Request Sent");
  if (client.call(srv)) {
    ROS_INFO("Object Successfully Deleted");
  } else {
    ROS_WARN("%s", srv.response.info.c_str());
    return 1;
  }
  return 0;
}
