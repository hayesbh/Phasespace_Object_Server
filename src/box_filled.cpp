/**
 * File: box_filled.cpp
 * Author: Dylan Visher
 * Date: 5/18/13
 * About: ROS client for the box_filled server
 * Checks whether box at given x,y,z and width is filled
 * And returns which ones are
 */

#include <cstdlib>
#include <vector>
#include "ros/ros.h"
#include "Phasespace_Object_Server/box_filled.h"

using std::vector;
using std::string;

int main(int argc, char **argv) {
  /*itintialize the ROS client*/
  ros::init(argc, argv, "box_filled");
  /*Check for inproper number of arguments*/
  if (argc != 5) {
    ROS_WARN("usage: rosrun Phasespace_Object_Server box_filled x y z width");
    return 1;
  }
  /*Initialize the service client Node on the add_object service*/
  ros::NodeHandle n;
  ros::ServiceClient client =
    n.serviceClient<Phasespace_Object_Server::box_filled>("box_filled");
  Phasespace_Object_Server::box_filled srv;
  srv.request.x = atof(argv[1]);
  srv.request.y = atof(argv[2]);
  srv.request.z = atof(argv[3]);
  srv.request.width = atof(argv[4]);
  /*If the call was successful*/
  ROS_INFO("Request Sent");
  if (client.call(srv)) {
    if(srv.response.filled) {
      ROS_INFO("The Box is filled by objects: ");
      vector<string> objects = srv.response.objects;
      vector<string>::iterator iter;
      for(iter = objects.begin(); iter != objects.end(); ++iter) {
        ROS_INFO("%s", iter->c_str());
      }
    } else {
      ROS_INFO("The Box is empty");
    }
  } else {
    ROS_WARN("The Service Call Failed\n");
    return 1;
  }
  return 0;
}
