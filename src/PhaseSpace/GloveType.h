// File: GloveType.h
// Author: Dylan Visher
// Date: 5/18/13
// About: Glove extends Object Type adding in more specific "Hand" information

#ifndef _SHL_COREOBJECTSERVER_GLOVETYPE_H
#define _SHL_COREOBJECTSERVER_GLOVETYPE_H

#include <vector>
#include "ObjectType.h"
#include "Point.h"

namespace object_server {

using std::vector;
using object_server::ObjectType;

class GloveType : public ObjectType {
private:
	vector<Point>::iterator thumb;
	vector<Point>::iterator fore;
	vector<Point>::iterator middle;
	vector<Point>::iterator ring;
	vector<Point>::iterator pinkey;
	vector<Point>::iterator base_left;
	vector<Point>::iterator base_right;
public:
  // Initialize the glove and all the finger Points
  void init(vector<Point> p);
  Point get_pointer();
  Point GetFirstAxis(int i=0);
  Point GetSecondAxis(int i=0);
};

} //object_server

#endif
