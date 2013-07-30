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


class ObjectType {
  protected:
    // Location Information
    vector<Point> points_;
    Point center_;
    bool rigid_;
    // Angle Information
    // Original Axis is the original vector for Axis1
    Point original_axis1_;
    // A vector of iterators that point to the two consituent points_ of Axis1
    vector<vector<Point>::iterator> axis_points1_;
    // The Current vector describing Axis1
    Point axis1_;
    // The Second Axis for Another Axis of Rotation Information
    Point original_axis2_;
    vector<vector<Point>::iterator> axis_points2_;
    Point axis2_;
    // Quaternion for holding the rotational information
    vector<float> angle_;
    // x, y, and z scale for holding the dim_ensional information
    vector<float> dim_;
  public:
    virtual bool Init(vector<Point> p, bool rig)=0;
    vector<int> get_axis1_ids() {
      vector<int> ids;
      if (axis_points1_.size() != 2) return ids;
      ids.push_back(axis_points1_[0]->id_);
      ids.push_back(axis_points1_[1]->id_);
      return ids;
    }
    Point get_original_axis1(){
      return original_axis1_;
    }
    vector<int> get_axis2_ids() {
      vector<int> ids;
      if (axis_points2_.size() != 2) return ids;
      ids.push_back(axis_points2_[0]->id_);
      ids.push_back(axis_points2_[1]->id_);
      return ids;
    }
    Point get_original_axis2() {
      return original_axis2_;
    }
    bool SetAxis1IDs(vector<int> ids) {
      vector<Point>::iterator p1;
      if((p1 = FindPointById(ids[0], points_)) == points_.end())
        return false;
      vector<Point>::iterator p2;
      if((p2 = FindPointById(ids[1], points_)) == points_.end())
        return false;
      axis_points1_.clear();
      axis_points1_.push_back(p1);
      axis_points1_.push_back(p2);
      return true;
    }
    bool SetAxis2IDs(vector<int> ids) {
      vector<Point>::iterator p1;
      if((p1 = FindPointById(ids[0], points_)) == points_.end())
        return false;
      vector<Point>::iterator p2;
      if((p2 = FindPointById(ids[1], points_)) == points_.end())
        return false;
      axis_points2_.clear();
      axis_points2_.push_back(p1);
      axis_points2_.push_back(p2);
      return true;
    }
    bool SetOriginalAxis1(Point p) {
      original_axis1_ = p;
      return true;
    }
    bool SetOriginalAxis2(Point p) {
      original_axis2_ = p;
      return true;
    }
    bool SetDimensions(float x, float y, float z) {
      dim_.clear();
      dim_.push_back(x);
      dim_.push_back(y);
      dim_.push_back(z);
      return true;
    }
    bool SetRigid(bool rig) {
      rigid_ = rig;
      return true;
    }
    // Reset the object from the start
    virtual void reset();
    virtual bool get_rigidity() {
      return rigid_;
    }
    // get_dim_ensions returns the dimensional information of the object
    virtual vector<float> get_dimensions() {
      return dim_;
    }
    virtual vector<float> get_angle() {
      vector<float> default_angle;
      default_angle.push_back(1);
      default_angle.push_back(0);
      default_angle.push_back(0);
      default_angle.push_back(0);
      if(angle_.size() == 0) return default_angle;
      return angle_;
    }
    // get_axes returns the axis information of the object
    virtual vector<Point> GetAxes();
    // get_pointer returns the pointer of the object
    // return a Point representing 3D coordinate of the point
    virtual Point get_pointer()=0;
    // get_points_ return the points_ that define the object
    // return these points_ in a vector
    virtual vector<Point> get_points() {
      return points_;
    }
    // get_center_ return the Point that defines the center
    // return the center_ Point
    virtual Point get_center() {
      return center_;
    }
    // updates the points_ information
    virtual void Update(OWLMarker *markers, int n);
    virtual bool update();
    // MaxDimensionalDistance find the dim_ensions along the local axis
    // dim_ension: int representing axis (0 --> x axis, 1 --> y axis, 2 --> z axis)
    // return  float giving the largest distance from the center_ along that axis
    virtual float MaxDimensionalDistance(int dimension);
    // GetScale finds the dim_ensions of the object
    virtual void GetScale(int i=0);
    // AddPoints adds new_points_ to the object
    // new_points_ is a list of Points that are to be added to the Object
    virtual bool AddPoints(vector<Point> new_points);
    // GetCenter sets the center_ based off of the information 
    virtual void GetCenter(int i=0);
    // GetFirstAngleAxis defines the x_axis of the object specific frame of reference
    virtual bool GetFirstAxis(int i=0)=0;
    // GetSecondAngleAxis defines the y_axis of the object specific frame of reference
    virtual bool GetBothAxes(int i=0)=0;
    // GetAngle defines the angle_ of the Object from its OriginalAxes
    virtual void GetAngle(int i=0);
};
}  // namespace object_server
#endif
