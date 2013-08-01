// File: Object.h
// Author: Dylan Visher
// Date: 7/18/13
// About: Virtual General Object Class

#ifndef _SHL_OBJECT_SERVER_SRC_PHASESPACE_OBJECT_H_
#define _SHL_OBJECT_SERVER_SRC_PHASESPACE_OBJECT_H_
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
    // SetDimensions sets the dimensions of the Object
    // x: the object's scale along the local x_axis
    // y: the object's scale along the local y_axis
    // z: the object's scale along the local z_axis
    // return whether these dimensions were properly set
    bool SetDimensions(float x, float y, float z);
    
    // get_id returns the id of the object
    // return int representing the identification number
    int get_id() {
      return id_;
    }
    // get_type returns the type of object
    // return string i.e. manual, default, glove
    //   where manual indicates that it is a manual object
    //         default indicates that it is a default PSObject
    //         glove indicates that it is a Glove PSObject
    virtual string get_type()=0;
    // get_rigidity returns the rigidity
    // return: bool (true = rigid, false = not rigid)
    virtual bool get_rigidity()=0;
    // get_name returns the name of the object
    // return a string representing the user given name
    string get_name(){
      return name_;
    }
    // get_pointer returns a Point that represents where this object is pointing
    virtual Point get_pointer()=0;
    // CollidesWith determines whether this object collides with another object
    // obj: the Object to check the collision with
    virtual bool CollidesWith(Object* obj);
    // get_center returns the center of the object
    // return a Point that represents the center of the object
    Point get_center() {
      return center_;
    }
    // get_rotation returns the rotation of the object
    // return vector of floats (a quaternion) representing rotation
    vector<float> get_rotation() {
      return angle_;
    }
    // get_axes returns the axes of the object
    // return a vector of Points representing the unit axes
    //  (X, Y, Z) where X is the local x axis etc.
    vector<Point> get_axes() {
      return axes_;
    }
    // get_dimensions returns the dimension (extents) of the object
    // return a 3D array of floats descibing the local dimensions of the object
    vector<float> get_dimensions() {
      return dim_;
    }
    // return the points that make up the object
    // default is just to return the empty vector
    virtual vector<Point> get_points() {
      vector<Point> p;
      return p;
    }
    // AddPoints adds the points to the object
    // return: bool indicating whether any points were succesfully added
    virtual bool AddPoints(vector<Point> new_points)=0;
    // Update updates points with new marker information
    // and Updates the Fields based of new marker information
    virtual bool Update(OWLMarker* marks, int n)=0;
    // UpdateFields updates the Fields of the object dimension, axes, angle, center
    virtual bool UpdateFields()=0;
};

const vector<Object*>::iterator FindObjectByName(string name, vector<Object*> &objects);

}  // namespace object_server
#endif  // _SHL_OBJECT_SERVER_SRC_PHASESPACE_OBJECT_H_
