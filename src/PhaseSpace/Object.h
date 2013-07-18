// File: Object.h
// Author: Dylan Visher
// Date: 7/18/13
// About: General Object Class

#ifndef _SHL_COREOBJECTSERVER_OBJECT_H
#define _SHL_COREOBJECTSERVER_OBJECT_H
#include <string>
#include <vector>
#include "./Point.h"

namespace object_server {

using object_server::Point;
using std::string;

class Object {
  protected:
    // Tracking Information
    int id;  // For the system to keep track of the Object
    string name;  // For the user to keep track of the Object
    string ext;  // To determing what type of Object extension it is PS or Manual
    // State Information
    Point center;  // The center location (X, Y, Z)
    vector<float> angle;  // The rotation from its original position
    vector<Point> axes; // The axes of the object
    vector<float> dim; // The extents of the object

  public:
    virtual void init();
    // get_id returns the id of the object
    // return int representing the identification number
    int get_id() {
      return id;
    }
    // get_name returns the name of the object
    // return a string representing the user given name
    string get_name(){
      return name;
    }
    // get_object type returns the type of the object
    // return a string representing describing type
    string get_object_ext() {
      return ext;
    }
    // get_pointer returns a Point that represents where this object is pointing
    virtual Point get_pointer();
    // CollidesWith determines whether this object collides with another object
    virtual bool CollidesWith(Object obj);
    // get_center returns the center of the object
    // return a Point
    Point get_center() {
      return center;
    }
    // get_rotation returns the rotation of the object
    // return a quaternion representing rotation
    vector<float> get_rotation() {
      return angle;
    }
    // get_axes returns the axes of the object
    // return a vector of floats representing the unit axes
    vector<Point> get_axes() {
      return axes;
    }
    // get_dimensions returns the dimension (extents) of the object
    // return a 3D array of floats
    vector<float> get_dimensions() {
      return dim;
    }
    // Update Updates the information inside the Object
    virtual bool Update();
};

}  // namespace object_server
#endif      