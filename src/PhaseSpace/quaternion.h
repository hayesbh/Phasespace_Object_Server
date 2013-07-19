// File: quaternion.h
// Author: Dylan Visher
// Date: 5/18/13
// About: Quaternions

#ifndef _SHL_COREOBJECTSERVER_QUATERNION_H
#define _SHL_COREOBJECTSERVER_QUATERNION_H

#include <vector>
#include "Point.h"


namespace quaternions {

using std::vector;
using object_server::Point;

vector<float> Qmult(vector<float> v1, vector<float> v2);
vector<float> Qconj(vector<float> v);
float Qnorm(vector<float> v);
vector<float> Qnormalize(vector<float> v);
vector<float> Qinv(vector<float> v);
Point QRotate(Point p, vector<float> q);

}  // namespace quaternions

#endif
