/**
 * File: Glove.h
 * Author: Dylan Visher
 * Date: 5/18/13
 * About: Glove extends Object Type adding in more specific "Hand" information
 */
#ifndef _SHL_COREOBJECTSERVER_GLOVE_H
#define _SHL_COREOBJECTSERVER_GLOVE_H

namespace object_server {

using object_server::ObjectType;

class Glove : public ObjectType {
private:
	vector<Point>::iterator thumb;
	vector<Point>::iterator fore;
	vector<Point>::iterator middle;
	vector<Point>::iterator ring;
	vector<Point>::iterator pinkey;
	vector<Point>::iterator base_left;
	vector<Point>::iterator base_right;
public:
  void init(vector<Point> p) {
    vector<Point>::iterator iter;
    /* Push back each of the points and blank ones where they are needed */
    points.insert(points.end(), p.begin(), p.end());
    /* Set up finger specific pointers */
    for (iter = points.begin(); iter != points.end(); ++iter) {
      if ( iter->id % 7 == 1 ) thumb = iter;
      else if ( iter->id % 7 == 2 ) base_left = iter;
      else if ( iter->id % 7 == 3 ) fore = iter;
      else if ( iter->id % 7 == 4 ) middle = iter;
      else if ( iter->id % 7 == 0 ) ring = iter;
      else if ( iter->id % 7 == 5 ) pinkey = iter;
      else if ( iter->id % 7 == 6 ) base_right = iter;
    }
    GetCenter();
    GetFirstAngleAxis();
    GetAngle();
    GetScale();
  }
  Point GetFirstAxisAngle() {
    if (AngleAxis1.size() == 2) {
      if(base_right->current == 0 || base_left->current == 0) {
        return vAngleAxis1;
      }
      vAngleAxis1 = base_right->sub(*base_left).normalize();
      return vAngleAxis1;
    } else {
      AngleAxis1.push_back(base_left);
      AngleAxis1.push_back(base_right);
      OriginalAxis1 = base_right->sub(*base_left).normalize();
      vAngleAxis1 = OriginalAxis1;
      return vAngleAxis1;
    }
  }
  Point GetSecondAngleAxis() {
    if (AngleAxis2.size() == 2) {
     /*check first axis*/
      if(base_left->current == 0 || base_right->current == 0) {
        return vAngleAxis2;
      }
      /*check second axis*/
      Point u;
      if(fore->current == 0) {
        return vAngleAxis2;
      } else {
        u = fore->sub(*base_left);
      }
      Point v = fore->sub(*base_left);
      vAngleAxis2 = u.sub(v.normalize().times(u.dot(v))).normalize();
      // Now find the normal component
      return vAngleAxis2;
   }
    else {
      AngleAxis2.push_back(fore);
      AngleAxis2.push_back(base_left);
      Point v = fore->sub(*base_left);
      vAngleAxis2 =  v.sub(vAngleAxis1.times(v.dot(vAngleAxis1))).normalize();
      OriginalAxis2 = vAngleAxis2;
      return vAngleAxis2;
    }
  }
  
};

} //object_server

#endif
