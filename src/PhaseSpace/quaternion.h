/**
 * File: Point.h
 * Author: Dylan Visher
 * Date: 5/18/13
 * About: Library for Quaternions
 */

#ifndef _SHL_COREOBJECTSERVER_QUATERNION_H
#define _SHL_COREOBJECTSERVER_QUATERNION_H

namespace quaternions {

#include <vector>
#include "./Point.h"
using std::vector;
using object_server::Point;

vector<float> Qmult(vector<float> v1, vector<float> v2) {
  Point p1; p1.init(v1[1], v1[2], v1[3]);
  Point p2; p2.init(v2[1], v2[2], v2[3]);
  vector<float> result;
  result.push_back(v1[0]*v2[0] - p1.dot(p2));
  Point vec;
  vec = p2.times(v1[0]).add(p1.times(v2[0])).add(p1.cross(p2));
  result.push_back(vec.x);
  result.push_back(vec.y);
  result.push_back(vec.z);
  return result;
}
vector<float> Qconj(vector<float> v) {
  vector<float> conj;
  conj.push_back(v[0]);
  conj.push_back(-1*v[1]);
  conj.push_back(-1*v[2]);
  conj.push_back(-1*v[3]);
  return conj;
}
float Qnorm(vector<float> v) {
  return sqrt(pow(v[0], 2) + pow(v[1], 2) + pow(v[2], 2) + pow(v[3], 2));
}
vector<float> Qnormalize(vector<float> v) {
  vector<float>::iterator iter;
  float mag = Qnorm(v);
  for(iter = v.begin(); iter != v.end(); ++iter) {
  	*iter = *iter / mag;
  } return v;
}
vector<float> Qinv(vector<float> v) {
  float divisor = pow(Qnorm(v), 2);
  vector<float> inverse = Qconj(v);
  inverse[0] /= divisor;
  inverse[1] /= divisor;
  inverse[2] /= divisor;
  inverse[3] /= divisor;
  return inverse;
}
Point QRotate(Point p, vector<float> q) {
	q = Qnormalize(q);
	Point u;
	u.init(q[1], q[2], q[3]);
	float s = q[0];
	return u.times(2.0f * u.dot(p)).add(p.times(s*s - u.dot(u))).add(u.cross(p).times(s * 2.0f));
}


}  // namespace quaternions

#endif