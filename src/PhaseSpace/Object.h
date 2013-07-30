// File: Object.h
// Author: Dylan Visher
// Date: 7/18/13
// About: Virtual General Object Class

#ifndef _SHL_COREOBJECTSERVER_OBJECT_H
#define _SHL_COREOBJECTSERVER_OBJECT_H
#include <string>
#include <vector>
#include "./Point.h"
#include <math.h>

namespace object_server {

using std::string;

class Object {
  protected:
    // Tracking Information
    int id_;  // For the system to keep track of the Object
    string name_;  // For the user to keep track of the Object
    // State Information
    Point center_;  // The center location (X, Y, Z)
    vector<float> angle_;  // The rotation from its original position
    vector<Point> axes_; // The axes of the object
    vector<float> dim_; // The extents of the object

  public:
    //virtual void init()=0;
    // get_id returns the id of the object
    // return int representing the identification number
    int get_id() {
      return id_;
    }
    virtual string get_type()=0;
    virtual bool get_rigidity()=0;
    // get_name returns the name of the object
    // return a string representing the user given name
    string get_name(){
      return name_;
    }
    // get_pointer returns a Point that represents where this object is pointing
    virtual Point get_pointer()=0;
    // CollidesWith determines whether this object collides with another object
    virtual bool CollidesWith(Object* obj);
    // get_center returns the center of the object
    // return a Point
    Point get_center() {
      return center_;
    }
    // get_rotation returns the rotation of the object
    // return a quaternion representing rotation
    vector<float> get_rotation() {
      return angle_;
    }
    // get_axes returns the axes of the object
    // return a vector of floats representing the unit axes
    vector<Point> get_axes() {
      return axes_;
    }
    // get_dimensions returns the dimension (extents) of the object
    // return a 3D array of floats
    vector<float> get_dimensions() {
      return dim_;
    }
    virtual vector<Point> get_points() {
      vector<Point> p;
      return p;
    }
    virtual bool AddPoints(vector<Point> new_points)=0;
    virtual bool Update(OWLMarker* marks, int n)=0;
    virtual bool UpdateFields()=0;
};

const vector<Object*>::iterator FindObjectByName(string name, vector<Object*> &objects);

}  // namespace object_server
#endif      
