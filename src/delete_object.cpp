/**
 * File: delete_object.cpp
 * Author: Dylan Visher
 * Date: 5/18/13
 * About: ROS client for the delete_object server
 * Delete object from the tracked
 */
#include <cstdlib>
#include "ros/ros.h"
#include "core_object_server/delete_object.h"

int main(int argc, char **argv) {
  /*initialize ROS*/
  ros::init(argc, argv, "add");
  /*check for proper number of arguments*/
  if (argc != 2) {
    ROS_WARN("usage: rosrun test delete_object name");
    return 1;
  }
  /*create the service client node on the delete_object service*/
  ros::NodeHandle n;
  ros::ServiceClient client =
    n.serviceClient<core_object_server::delete_object>("delete_object");
  core_object_server::delete_object srv;
  srv.request.id = atol(argv[1]);
  if (client.call(srv)) {
    ROS_INFO("Request Sent");
    if (!(static_cast<int>(srv.response.success)))
      ROS_WARN("Delete Object: %i failed", srv.request.id);
    else
      ROS_INFO("Object Successfully Deleted");
  } else {
    ROS_ERROR("Failed to call service");
    return 1;
  }
  return 0;
}
