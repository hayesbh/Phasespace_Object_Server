/**
 * File: collides.cpp
 * Author: Dylan Visher
 * Date: 5/18/13
 * About: ROS client for the collides service
 * Checks whether one object is colliding with another
 * Returns bool indicating truth
 */

#include <cstdlib>
#include <vector>
#include "ros/ros.h"
#include "core_object_server/collides.h"

using std::vector;
using std::string;

int main(int argc, char **argv) {
  /*itintialize the ROS client*/
  ros::init(argc, argv, "collides");
  /*Check for inproper number of arguments*/
  if (argc != 3) {
    ROS_WARN("usage: rosrun core_object_server collides name1 name2");
    return 1;
  }
  /*Initialize the service client Node on the add_object service*/
  ros::NodeHandle n;
  ros::ServiceClient client =
    n.serviceClient<core_object_server::collides>("collides");
  core_object_server::collides srv;
  srv.request.name1 = argv[1];
  srv.request.name2 = argv[2];
  /*If the call was successful*/
  ROS_INFO("Request Sent");
  if (client.call(srv)) {
    if(srv.response.collides) {
      ROS_INFO("Collision Detected");
    } else {
      ROS_INFO("No Collision Detected");
    }
  } else {
    ROS_WARN("The Service Call Failed\n");
    return 1;
  }
  return 0;
}
