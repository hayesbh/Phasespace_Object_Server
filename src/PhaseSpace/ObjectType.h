// File: ObjectType.h
// Author: Dylan Visher
// Date: 5/18/13
// About: ObjectType contains type specific information

#ifndef _SHL_OBJECT_SERVER_SRC_PHASESPACE_OBJECTTYPE_H_
#define _SHL_OBJECT_SERVER_SRC_PHASESPACE_OBJECTTYPE_H_

#include <vector>
#include <cstdio>
#include <math.h>
#include "./Point.h"
#include "./quaternion.h"

namespace Phasespace_Object_Server {


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
    // A vector of iterators that point to the two constituent Point's of points_ in Axis2
    vector<vector<Point>::iterator> axis_points2_;
    // The Current Vector for describing Axis1
    Point axis2_;
    // Quaternion for holding the rotational information
    vector<float> angle_;
    // x, y, and z scale for holding the dim_ensional information
    vector<float> dim_;
  public:
    // Init sets initializes the points_ of the glove
    // finds all of the fingers' specific Points
    // Finds the center, the angle_ and the scale
    // p: is the list of points for the glove
    // rig: is the rigidity of the glove
    virtual bool Init(vector<Point> p, bool rig)=0;
    // get_axis1_ids gets the returns a vector of the ids of the first axis
    // return: vector of ints representing Point ids
    vector<int> get_axis1_ids() {
      vector<int> ids;
      if (axis_points1_.size() != 2) return ids;
      ids.push_back(axis_points1_[0]->id_);
      ids.push_back(axis_points1_[1]->id_);
      return ids;
    }
    // get_original_axis1 returns the Point vector describing the original first axis
    Point get_original_axis1(){
      return original_axis1_;
    }
    // get_axis2_ids returns a vector of ids that define the second axis
    // return: vector of ints representing Point ids
    vector<int> get_axis2_ids() {
      vector<int> ids;
      if (axis_points2_.size() != 2) return ids;
      ids.push_back(axis_points2_[0]->id_);
      ids.push_back(axis_points2_[1]->id_);
      return ids;
    }
    // get_original_axis2 returns the Point vector describing the original seconds axis
    Point get_original_axis2() {
      return original_axis2_;
    }
    // SetAxis1IDs sets the Ids in the first axis to be those given
    // ids: a vector of ids to use as the first axis for this object
    // return bool representing whether these were successfully added
    bool SetAxis1IDs(vector<int> ids) {
      vector<Point>::iterator p1;
      // make sure that both the id's indicate points in this object
      // find their respective iterators
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
    // SetAxis2IDs sets the Ids in the second axis to be those given
    // ids: a vector of ids to use as the second axis for this object
    // return bool representing whether these were successfully added
    bool SetAxis2IDs(vector<int> ids) {
      // Make sure the points are actually within this object
      // find their respecive iterators
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
    // SetOriginalAxis1 sets the original_axis1 to be a certain vector
    // p: is what the original_axis1 will be set as
    // return whether this was successful
    bool SetOriginalAxis1(Point p) {
      original_axis1_ = p;
      return true;
    }
    // SetOriginalAxis2 sets the original_axis2 to be a certain vector
    // p: is what the original_axis2 will be set as
    // return whether this was successful
    bool SetOriginalAxis2(Point p) {
      original_axis2_ = p;
      return true;
    }
    // SetDimensions sets the dimensions(dim) of the object
    // This will be overwritten if object is not Rigid
    // x: the new x_scale
    // y: the new y_scale
    // z: the new z_scale
    bool SetDimensions(float x, float y, float z) {
      dim_.clear();
      dim_.push_back(x);
      dim_.push_back(y);
      dim_.push_back(z);
      return true;
    }
    // SetRigid sets the rigidity of the Object
    // rig: bool indicating the new rigidity of the object
    // return: bool indicating that this rigidity was successfully changed
    bool SetRigid(bool rig) {
      rigid_ = rig;
      return true;
    }
    // Reset the object from the start
    // I.E. ReSet the Axes, Reset the Angles, Reset the Dimensions
    virtual void reset();
    // get_rigidity returns the rigidity of the object
    // return: bool indicating rigidity (true means rigid)
    virtual bool get_rigidity() {
      return rigid_;
    }
    // get_dimensions returns the dimensional information of the object
    // return vector of dimensions (x, y, z)
    virtual vector<float> get_dimensions() {
      return dim_;
    }
    // get_angle returns the angle information of the Object
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
    // UpdateFields calls all of the functions for updating Fields such as axis information, angle, and dimensions
    virtual bool UpdateFields();
    // MaxDimensionalDistance find the dim_ensions along the local axis
    // dim_ension: int representing axis (0 --> x axis, 1 --> y axis, 2 --> z axis)
    // return  float giving the largest distance from the center_ along that axis
    virtual float MaxDimensionalDistance(int dimension);
    // GetScale finds the dimensions of the object
    // This is determined by 10% longer then the furthest distance along each local axis
    // Where the distance is distance from center to furthest out point
    virtual void GetScale(int i=0);
    // AddPoints adds new_points_ to the object
    // new_points_ is a list of Points that are to be added to the Object
    virtual bool AddPoints(vector<Point> new_points);
    // GetCenter sets the center_ based off of the points
    virtual void GetCenter(int i=0);
    // GetFirstAngleAxis finds (and sets if first time of i == 1) the local x_axis
    // default first angle axis is defined by the two points on the base of the hand (base_left -> base_right)
    // the vector points in the +x direction
    // i: default set to 0 but if it is ever set to 1 the first axis will be reset according to the rules above
    // return: bool indicating whether this was successful at getting the first axis
    virtual bool GetFirstAxis(int i=0)=0;
    // GetBothAxes ginds both Axes (calls on GetFirstAxis) and sets both (if the first time or i == 1)
    // this is the local y axis and is defined by the base_left and pointer marker
    // these axes that are initialized define the angle of the object
    // i: default set to 0 but if it is ever set to 1 the second axis and first axis will be reset according to the rules above
    // return: bool that indicates whether both axes were set
    virtual bool GetBothAxes(int i=0)=0;
    // GetAngle defines the angle_ of the Object from its OriginalAxes
    // This just runs the GetAxes(0) and compares them against the original
    virtual void GetAngle(int i=0);
};
}  // namespace Phasespace_Object_Server
#endif  // _SHL_OBJECT_SERVER_SRC_PHASESPACE_OBJECTTYPE_H_
