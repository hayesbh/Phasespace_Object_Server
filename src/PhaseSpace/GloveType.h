// File: GloveType.h
// Author: Dylan Visher
// Date: 5/18/13
// About: Glove extends Object Type adding in more specific "Hand" information

#ifndef _SHL_OBJECT_SERVER_SRC_PHASESPACE_GLOVETYPE_H_
#define _SHL_OBJECT_SERVER_SRC_PHASESPACE_GLOVETYPE_H_

#include <vector>
#include "./ObjectType.h"
#include "./Point.h"

namespace Phasespace_Object_Server {

using std::vector;
using Phasespace_Object_Server::ObjectType;

class GloveType : public ObjectType {
  public:
    vector<Point>::iterator thumb_;
    vector<Point>::iterator fore_;
    vector<Point>::iterator middle_;
    vector<Point>::iterator ring_;
    vector<Point>::iterator pinkey_;
    vector<Point>::iterator base_left_;
    vector<Point>::iterator base_right_;
    // Initialize the glove and all the finger Points with given rigidity
    bool Init(vector<Point> p, bool rig);
    // get_pointer returns the pointer finger
    // return the point on the pointer finger
    Point get_pointer();
    // GetFirstAxis sets the first axis of the hand (the base of the hand)
    // return success in finding
    bool GetFirstAxis(int i = 0);
    // GetSecondAxis sets the second axis of the hand (pointer and left base)
    // return true if it was able to update the axis information (the Points were current)
    bool GetBothAxes(int i = 0);
    // AddPoints for a Glove always returns false, nothing else can be added to this set object
    bool AddPoints() {
      return false;
    }
};
}  // Phasespace_Object_Server
#endif  // _SHL__OBJECT_SERVER_SRC_PHASESPACE_GLOVETYPE_H_
