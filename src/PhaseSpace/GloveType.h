// File: GloveType.h
// Author: Dylan Visher
// Date: 5/18/13
// About: Glove extends Object Type adding in more specific "Hand" information

#ifndef _SHL__OBJECT_SERVER_SRC_PHASESPACE_GLOVETYPE_H_
#define _SHL__OBJECT_SERVER_SRC_PHASESPACE_GLOVETYPE_H_

#include <vector>
#include "./ObjectType.h"
#include "./Point.h"

namespace object_server {

using std::vector;
using object_server::ObjectType;

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
    // return success in finding
    bool GetBothAxes(int i = 0);
    bool AddPoints() {
      return false;
    }
};
}  // object_server
#endif  // _SHL__OBJECT_SERVER_SRC_PHASESPACE_GLOVETYPE_H_
