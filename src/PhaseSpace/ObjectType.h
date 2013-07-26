// File: ObjectType.h
// Author: Dylan Visher
// Date: 5/18/13
// About: ObjectType contains type specific information

#ifndef _SHL_COREOBJECTSERVER_H
#define _SHL_COREOBJECTSERVER_H

#include <vector>
#include <cstdio>
#include <math.h>
#include "./Point.h"
#include "./quaternion.h"

namespace object_server {

using object_server::Point;

class ObjectType {
  protected:
    // Location Information
    vector<Point> points;
    Point center;
    bool rigid;
    // Angle Information
    // Original Axis is the original vector for Axis1
    Point OriginalAxis1;
    // A vector of iterators that point to the two consituent points of Axis1
    vector<vector<Point>::iterator> AxisPoints1;
    // The Current vector describing Axis1
    Point Axis1;
    // The Second Axis for Another Axis of Rotation Information
    Point OriginalAxis2;
    vector<vector<Point>::iterator> AxisPoints2;
    Point Axis2;
    // Quaternion for holding the rotational information
    vector<float> angle;
    // x, y, and z scale for holding the dimensional information
    vector<float> dim;
  public:
    virtual bool init(vector<Point> p, bool rig)=0;
    vector<int> get_axis1_ids() {
      vector<int> ids;
      if (AxisPoint1.size() != 2) return ids;
      ids.push_back(AxisPoints1[0]->id);
      ids.push_back(AxisPoints1[1]->id);
      return ids;
    }
    vector<int> get_axis2_ids() {
      vector<int> ids;
      if (AxisPoint2.size() != 2) return ids;
      ids.push_back(AxisPoints2[0]->id);
      ids.push_back(AxisPoints2[1]->id);
      return ids;
    }
    bool SetAxis1IDs(vector<int> ids) {
      vector<Point>::iterator p1;
      if((p1 = FindById(ids[0], points)) == points.end())
        return false;
      vector<Point>::iterator p2;
      if((p2 = FindById(ids[1], points)) == points.end())
        return false;
      AxisPoints1.clear();
      AxisPoints1.push_back(p1);
      AxisPoints1.push_back(p2);
      return true;
    }
    bool SetAxis2IDs(vector<int> ids) {
      vector<Point>::iterator p1;
      if((p1 = FindById(ids[0], points)) == points.end())
        return false;
      vector<Point>::iterator p2;
      if((p2 = FindById(ids[1], points)) == points.end())
        return false;
      AxisPoints2.clear();
      AxisPoints2.push_back(p1);
      AxisPoints2.push_back(p2);
      return true;
    }
    bool SetOriginalAxis1(Point p) {
      OriginalAxis1 = p;
      return true;
    }
    bool SetOriginalAxis2(Point p) {
      OriginalAxis2 = p;
      return true;
    }
    // Reset the object from the start
    virtual void reset();
    virtual bool get_rigidity() {
      return rigid;
    }
    // get_dimensions returns the dimensional information of the object
    virtual vector<float> get_dimensions() {
      return dim;
    }
    virtual vector<float> get_angle() {
      vector<float> default_angle;
      default_angle.push_back(1);
      default_angle.push_back(0);
      default_angle.push_back(0);
      default_angle.push_back(0);
      if(angle.size() == 0) return default_angle;
      return angle;
    }
    // get_axes returns the axis information of the object
    virtual vector<Point> GetAxes();
    // get_pointer returns the pointer of the object
    // return a Point representing 3D coordinate of the point
    virtual Point get_pointer()=0;
    // get_points return the points that define the object
    // return these points in a vector
    virtual vector<Point> get_points() {
      return points;
    }
    // get_center return the Point that defines the center
    // return the center Point
    virtual Point get_center() {
      return center;
    }
    // updates the points information
    virtual void Update(OWLMarker *markers, int n);
    // MaxDimensionalDistance find the dimensions along the local axis
    // dimension: int representing axis (0 --> x axis, 1 --> y axis, 2 --> z axis)
    // return  float giving the largest distance from the center along that axis
    virtual float MaxDimensionalDistance(int dimension);
    // GetScale finds the dimensions of the object
    virtual void GetScale(int i=0);
    // AddPoints adds new_points to the object
    // new_points is a list of Points that are to be added to the Object
    virtual bool AddPoints(vector<Point> new_points);
    // GetCenter sets the center based off of the information 
    virtual void GetCenter(int i=0);
    // GetFirstAngleAxis defines the x_axis of the object specific frame of reference
    virtual bool GetFirstAxis(int i=0)=0;
    // GetSecondAngleAxis defines the y_axis of the object specific frame of reference
    virtual bool GetBothAxes(int i=0)=0;
    // GetAngle defines the angle of the Object from its OriginalAxes
    virtual void GetAngle(int i=0);
};
}  // namespace object_server
#endif
