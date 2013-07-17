// File: ObjectType.h
// Author: Dylan Visher
// Date: 5/18/13
// About: ObjectType contains type specific information
#ifndef _SHL_COREOBJECTSERVER_OBJECTTYPE_H
#define _SHL_COREOBJECTSERVER_OBJECTTYPE_H

#include <vector>
#include <math.h>
#include "./Point.h"
#include "./quaternion.h"

namespace object_server {

// Math And Vectors
using std::vector;
using std::acos;
using std::fabs;
using std::pow;
// Points
using object_server::Point;
using points::PointsToPlane;
using points::FindById;
// Quaternions
using quaternions::Qmult;
using quaternions::Qnormalize;
using quaternions::Qinv;
using quaternions::QRotate;

// This class is the Default Object for holding type specific information
class ObjectType {
  protected:
    // Location Information
    vector<Point> points;
    Point center;
    // Angle Information
    // Original Axis is the original vector for Axis1
    Point OriginalAxis1;
    // A vector of iterators that point to the two consituent points of Axis1
    vector<vector<Point>::iterator> AngleAxis1;
    // The Current vector describing Axis1
    Point vAngleAxis1;
    // The Second Axis for Another Axis of Rotation Information
    Point OriginalAxis2;
    vector<vector<Point>::iterator> AngleAxis2;
    Point vAngleAxis2;
    // Quaternion for holding the rotational information
    vector<float> angle;
    // x, y, and z scale for holding the dimensional information
    float x_scale;
    float y_scale;
    float z_scale;

  public:
  // init initializes the Object to hold in it the points given
  // p: The points that this object will track
  void init(vector<Point> p) {
    points = p;
    // Get the Center
    GetCenter();
    // Get the Angle information
    GetAngle();
    // Get Dimensional information
    GetScale();
  }
  void reset(){
    // Get the Center
    GetCenter();
    // Get Reset the Angle information so this is the new 0
    GetAngle(1);
    // Get Dimensional Information
    GetScale();
  }
  // get_dimensions returns the dimensional information of the object
  // dim[3]: an array for holding the x, y, z dimensions
  //        of the object respectively
  void get_dimensions(float dim[3]) {
    dim[0] = x_scale;
    dim[1] = y_scale;
    dim[2] = z_scale;
  }
  // get_pointer returns the pointer of the object
  // return a Point representing 3D coordinate of the point
  Point get_pointer() {
    Point P;
    P.init(0,0,0);
    return P;
  }
  // get_points return the points that define the object
  // return these points in a vector
  vector<Point> get_points() {
    return points;
  }
  // get_center return the Point that defines the center
  // return the center Point
  Point get_center() {
    return center;
  }

  // get_angle return the stored rotation information
  // return an array representing the quaternion angle (w,x,y,z)
  vector<float> get_angle() {
    return angle;
  }
  // Update updates the Objects points with new OWL marker information
  //        then it refreshes other object information
  //        The center, the angle, and the scale
  // markers: An array of OWLMarkers with updated information
  // n: the number of markers that have been updated
  void Update(OWLMarker *markers, int n) {
    for (int i = 0; i < n; ++i) {
      vector<Point>::iterator iter;
      for (iter = points.begin(); iter != points.end(); ++iter) {
        if (markers[i].id == iter->id) {
          if (markers[i].cond > 0)
            iter->Update(markers[i]);
          else
            iter->current = 0;
        }
      }
    }
    // Update Center information
    GetCenter();
    // Get the Angle Information
    GetAngle();
    // Get the Dimensional Information
    GetScale();
    return;
  }
  // IntersectsBox checks whether this object intersects a cube
        // Uses the seperation of Axes theorem
        // Inspired by Dynamic Collision Detection using Oriented Bounding Boxes by David Eveberly
        // This is optomized for the seach algorithm, the general Object Collision can be seen below
  // C: the point that describes the center of the cube
  // width: the length of one side of the cube 
  // return bool : Does the Object Intersect the cube
  int IntersectsBox(Point C, float width) {
    Point A[3] = { vAngleAxis1, vAngleAxis2, vAngleAxis1.cross(vAngleAxis2) };
    Point D = center.sub(C);
    float a[3] = { x_scale/2, y_scale/2, z_scale/2 };
    float b[3] = { width/2, width/2, width/2 };
    float c[3][3] = {{0,0,0},{0,0,0},{0,0,0}};
    if ((a[0] +
        b[0]*(c[0][0] = fabs(A[0].x)) + b[1]*(c[0][1] = fabs(A[0].y)) + b[2]*(c[0][2] = fabs(A[0].z)))
        < fabs(A[0].dot(D)))
       return 0;
    if ((a[1] +
        b[0]*(c[1][0] = fabs(A[1].x)) + b[1]*(c[1][1] = fabs(A[1].y)) + b[2]*(c[1][2] = fabs(A[1].z)))
        < fabs(A[1].dot(D)))
       return 0;
    if ((a[2] +
        b[0]*(c[2][0] = fabs(A[2].x)) + b[1]*(c[2][1] = fabs(A[2].y)) + b[2]*(c[2][2] = fabs(A[2].z)))
        < fabs(A[2].dot(D)))
       return 0;
    if ((a[0] * c[0][0] + a[1] * c[1][0] + a[2] * c[2][0]) +
       b[0]
       < fabs(D.x)) return 0;
    if ((a[0] * c[0][1] + a[1] * c[1][1] + a[2] * c[2][1]) +
       b[1]
       < fabs(D.y)) return 0;
    if ((a[0] * c[0][2] + a[1] * c[1][2] + a[2] * c[2][2]) +
       b[2]
       < fabs(D.z)) return 0;
    if ((a[1] * c[2][0] + a[2] * c[1][0]) +
        (b[1] * c[0][2] + b[2] * c[0][1])
       < fabs(c[1][0] * A[2].dot(D) - c[2][0] * A[1].dot(D))) return 0;
    if ((a[1] * c[2][1] + a[2] * c[1][1]) +
        (b[0] * c[0][2] + b[2] * c[0][1])
       < fabs(c[1][0] * A[2].dot(D) - c[2][1] * A[1].dot(D))) return 0;
    if ((a[1] * c[2][2] + a[2] * c[1][2]) +
        (b[0] * c[0][1] + b[1] * c[0][0])
       < fabs(c[1][2] * A[2].dot(D) - c[2][2] * A[1].dot(D))) return 0;
    
    if ((a[0] * c[2][0] + a[2] * c[0][0]) +
        (b[1] * c[1][2] + b[2] * c[1][1])
       < fabs(c[2][0] * A[0].dot(D) - c[0][0] * A[2].dot(D))) return 0;
    if ((a[0] * c[2][1] + a[2] * c[0][1]) +
        (b[0] * c[1][2] + b[2] * c[1][0])
       < fabs(c[2][1] * A[0].dot(D) - c[0][1] * A[2].dot(D))) return 0;
    if ((a[0] * c[2][2] + a[2] * c[0][2]) +
        (b[0] * c[1][1] + b[1] * c[1][0])
       < fabs(c[2][2] * A[0].dot(D) - c[0][2] * A[2].dot(D))) return 0;
    
    if ((a[0] * c[1][0] + a[1] * c[0][0]) +
        (b[1] * c[2][2] + b[2] * c[2][1])
       < fabs(c[0][0] * A[1].dot(D) - c[1][0] * A[2].dot(D))) return 0;
    if ((a[0] * c[1][1] + a[1] * c[0][1]) +
        (b[0] * c[2][2] + b[2] * c[2][0])
       < fabs(c[0][0] * A[1].dot(D) - c[1][1] * A[2].dot(D))) return 0;
    if ((a[0] * c[1][2] + a[1] * c[0][2]) +
        (b[0] * c[2][1] + b[1] * c[2][0])
       < fabs(c[0][2] * A[1].dot(D) - c[1][2] * A[2].dot(D))) return 0;
    return 1;
  }
  // IntersectsObject asks whether this object intersects another given object
  // obj: The object that we are asking if this one intersects
  // return bool: Does this object intersect obj?
  int IntersectsObject(ObjectType obj) {
    Point A[3] = { vAngleAxis1, vAngleAxis2, vAngleAxis1.cross(vAngleAxis2) };
    // Get the dimensional and axis information of the other object
    float dim[3] = { 0, 0, 0 };
    obj.get_dimensions(dim);
    vector<Point> axes = obj.GetAxes();
    Point B[3] = { axes[0],
                   axes[1],
                   axes[2] };
    Point D = center.sub(obj.get_center());
    // The extents
    float a[3] = { x_scale/2, y_scale/2, z_scale/2 };
    float b[3] = { dim[0]/2, dim[1]/2, dim[2]/2 };
    // Matrix of Dot Procducts A[i] dot B[j] -> C[i][j]
    float c[3][3] = {{0,0,0},{0,0,0},{0,0,0}};
    if ((a[0] +
        b[0]*(c[0][0] = fabs(A[0].x)) + b[1]*(c[0][1] = fabs(A[0].y)) + b[2]*(c[0][2] = fabs(A[0].z)))
        < fabs(A[0].dot(D)))
       return 0;
    if ((a[1] +
        b[0]*(c[1][0] = fabs(A[1].x)) + b[1]*(c[1][1] = fabs(A[1].y)) + b[2]*(c[1][2] = fabs(A[1].z)))
        < fabs(A[1].dot(D)))
       return 0;
    if ((a[2] +
        b[0]*(c[2][0] = fabs(A[2].x)) + b[1]*(c[2][1] = fabs(A[2].y)) + b[2]*(c[2][2] = fabs(A[2].z)))
        < fabs(A[2].dot(D)))
       return 0;
    if ((a[0] * c[0][0] + a[1] * c[1][0] + a[2] * c[2][0]) +
       b[0]
       < fabs(B[0].dot(D)))
       return 0;
    if ((a[0] * c[0][1] + a[1] * c[1][1] + a[2] * c[2][1]) +
       b[1]
       < fabs(B[1].dot(D)) return 0;
    if ((a[0] * c[0][2] + a[1] * c[1][2] + a[2] * c[2][2]) +
       b[2]
       < fabs(B[2].dot(D)) return 0;
    if ((a[1] * c[2][0] + a[2] * c[1][0]) +
        (b[1] * c[0][2] + b[2] * c[0][1])
       < fabs(c[1][0] * A[2].dot(D) - c[2][0] * A[1].dot(D))) return 0;
    if ((a[1] * c[2][1] + a[2] * c[1][1]) +
        (b[0] * c[0][2] + b[2] * c[0][1])
       < fabs(c[1][0] * A[2].dot(D) - c[2][1] * A[1].dot(D))) return 0;
    if ((a[1] * c[2][2] + a[2] * c[1][2]) +
        (b[0] * c[0][1] + b[1] * c[0][0])
       < fabs(c[1][2] * A[2].dot(D) - c[2][2] * A[1].dot(D))) return 0;

    if ((a[0] * c[2][0] + a[2] * c[0][0]) +
        (b[1] * c[1][2] + b[2] * c[1][1])
       < fabs(c[2][0] * A[0].dot(D) - c[0][0] * A[2].dot(D))) return 0;
    if ((a[0] * c[2][1] + a[2] * c[0][1]) +
        (b[0] * c[1][2] + b[2] * c[1][0])
       < fabs(c[2][1] * A[0].dot(D) - c[0][1] * A[2].dot(D))) return 0;
    if ((a[0] * c[2][2] + a[2] * c[0][2]) +
        (b[0] * c[1][1] + b[1] * c[1][0])
       < fabs(c[2][2] * A[0].dot(D) - c[0][2] * A[2].dot(D))) return 0;

    if ((a[0] * c[1][0] + a[1] * c[0][0]) +
        (b[1] * c[2][2] + b[2] * c[2][1])
       < fabs(c[0][0] * A[1].dot(D) - c[1][0] * A[2].dot(D))) return 0;
    if ((a[0] * c[1][1] + a[1] * c[0][1]) +
        (b[0] * c[2][2] + b[2] * c[2][0])
       < fabs(c[0][0] * A[1].dot(D) - c[1][1] * A[2].dot(D))) return 0;
    if ((a[0] * c[1][2] + a[1] * c[0][2]) +
        (b[0] * c[2][1] + b[1] * c[2][0])
       < fabs(c[0][2] * A[1].dot(D) - c[1][2] * A[2].dot(D))) return 0;
    return 1;
}
  // MaxDimensionalDistance find the dimensions along the local axis
  // dimension: int representing axis (0 --> x axis, 1 --> y axis, 2 --> z axis)
  // return  float giving the largest distance from the center along that axis
  float MaxDimensionalDistance(int dimension) {
    float max = 0;
    Point axis;
    /*find what axis we are looking at*/
    if(dimension == 0) axis.init(1, 0, 0);
    else if(dimension == 1) axis.init(0, 1, 0);
    else axis.init(0, 0, 1);
    /*Rotate that axis to find the local axis*/
    //axis = QRotate(axis, Qinv(angle));
    /*Find the maximum amount a vector lies along that axis*/
    vector<Point>::iterator iter;
    for(iter = points.begin(); iter != points.end(); ++iter) {
      if(!(iter->current)) continue;
      float temp;
      if((temp = fabs(iter->sub(center).dot(axis))) > max)
        max = temp;
    } return max;
  }
  /**
   * [GetScale finds and sets the local x, y, and z scale of the object]
   */
  void GetScale() {
    float buffer = 1.10;
    x_scale = 2 * MaxDimensionalDistance(0) * buffer;
    y_scale = 2 * MaxDimensionalDistance(1) * buffer;
    z_scale = 2 * MaxDimensionalDistance(2) * buffer;
  }
  /**
   * [AddPoints adds new_points to the Object]
   * @param new_points [a list of Points]
   */
  void AddPoints(vector<Point> new_points) {
    vector<Point>::iterator i;
    points.insert(points.end(), new_points.begin(), new_points.end());
    GetFirstAngleAxis(1);
    GetSecondAngleAxis(1);
  }
  /**
   * [getCenter default algorithm for getting the center, through average
   * TODO: Think about looking at max and min x + max and min y + max and min z]
   */
  void GetCenter() {
    Point p;
    p.init();
    int pointsUsed = 0;
    /*The Default Center is the Average of the points*/
    vector<Point>::iterator iter;
    for (iter = points.begin(); iter != points.end(); ++iter) {
      if (iter->current) {
        p.x += iter->x;
        p.y += iter->y;
        p.z += iter->z;
        pointsUsed++;
       }
     }
     if (pointsUsed != 0) {
        p.x /= pointsUsed;
        p.y /= pointsUsed;
        p.z /= pointsUsed;
        center = p;
        return;
     } else {
        center = points[0];
        return;
     }
  }
  /**
   * [GetAngleAxis sets or finds the vector of the Long Axis that defines angle]
   * @return [this vector axis]
   */
  Point GetFirstAngleAxis(int i = 0){
    if (AngleAxis1.size() == 2 && i == 0) {
      if(AngleAxis1[0]->current == 0 || AngleAxis1[1]->current == 0) {
        return vAngleAxis1;
      }
      vAngleAxis1 = AngleAxis1[1]->sub(*AngleAxis1[0]).normalize();
      return vAngleAxis1;
    } else {
      /* If there are not enough points then just return the 0 vector */ 
      if (points.size() < 2) {
        vAngleAxis1.init();
        return vAngleAxis1;
      }
      AngleAxis1.clear();
      /* Max distance Squared to compare with */
      float maxdist2 = 0;
      /* P1 and P2 are the Points that define the vector */
      vector<Point>::iterator P1;
      vector<Point>::iterator P2;
      /*Iterators for going throught the points that define the object*/
      vector<Point>::iterator it1;
      vector<Point>::iterator it2;
      for (it1 = points.begin(); it1 != points.end(); ++it1) {
      if(!(it1->current)) continue;
        for (it2 = points.begin()+1; it2 != points.end(); ++it2) {
          if(!(it2->current)) continue;
            float dist2 = pow(it1->x - it2->x, 2)
                      + pow(it1->y - it2->y, 2)
                      + pow(it1->z - it2->z, 2);
          if (dist2 > maxdist2) {
            maxdist2 = dist2;
            P1 = it1;
            P2 = it2;
          }
        }
      }
      /* If we are looking for the Axis that defines the angle
         Make sure that the id's are set so that it can be consistent */
      AngleAxis1.push_back(P1);
      AngleAxis1.push_back(P2);
      OriginalAxis1 = P2->sub(*P1).normalize();
      vAngleAxis1 = OriginalAxis1;
      return vAngleAxis1;
    }
  }
  Point GetSecondAngleAxis(int i = 0){
    if(AngleAxis1.size() != 2) {
      Point p;
      p.init();
      return p;
    }
    if (AngleAxis2.size() == 2 && i == 0) {
      /*check first axis*/
      if(AngleAxis1[0]->current == 0 || AngleAxis1[1]->current == 0) {
        return vAngleAxis2;
      }
      Point u;
      /*check second axis*/
      if(AngleAxis2[0]->current == 0 || AngleAxis2[1]->current == 0) {
        return vAngleAxis2;
      } else {
        u = AngleAxis2[1]->sub(*AngleAxis2[0]);
      }
      GetAngle();
      Point v;
      if(AngleAxis1[1] == AngleAxis2[0]) {
        v = AngleAxis1[0]->sub(*AngleAxis1[1]);
      } else {
        v = AngleAxis1[1]->sub(*AngleAxis1[0]);
      }
      vAngleAxis2 = u.sub(v.normalize().times(u.dot(v))).normalize();
      // Now find the normal component
      return vAngleAxis2;
    } else {
      /* If there are not enough points then just return the 0 vector */ 
      if (points.size() < 3) {
        vAngleAxis2.init();
        return vAngleAxis2;
      }
      AngleAxis2.clear();
      /*check first axis*/
      if(AngleAxis1[0]->current == 0 || AngleAxis1[1]->current == 0) {
        vAngleAxis2.init();
        return vAngleAxis2;
      }
      /* Max distance to compare with */
      float maxdist = 0;
      /* P1 and P2 are the Points that define the vector */
      vector<Point>::iterator P1;
      vector<Point>::iterator P2;
      vector<Point>::iterator iter;
      vector<Point>::iterator axis[2] = { AngleAxis1[0], AngleAxis1[1] };
      for(int i = 0; i <= 1; ++i) {
        vector<Point> line;
        line.push_back(*axis[i]);
        for (iter = points.begin(); iter != points.end(); ++iter) {
          if(iter == axis[0] || iter == axis[1]) continue;
          if(!(iter->current)) continue;
          line.push_back(*iter);
          float dist = iter->VectorPerpendicularTo(line).magnitude();
          if (dist > maxdist) {
            maxdist = dist;
            P1 = axis[i];
            P2 = iter;
            OriginalAxis2 = P2->sub(*P1).normalize();
          } line.erase(line.end()-1, line.end());
        } line.clear();
      }
      /* If we are looking for the Axis that defines the angle
         Make sure that the id's are set so that it can be consistent */
      AngleAxis2.push_back(P1);
      AngleAxis2.push_back(P2);
      vAngleAxis2 = OriginalAxis2;
      return vAngleAxis2;
    }
  }
  /**
   * [GetAngle gets the Euler Angles of the Object]
   * @return [the Euler Angles]
   */
  /*http://www.vbforums.com/showthread.php?
  584390-Quaternion-from-two-3D-Position-Vectors*/
  void GetAngle(int i = 0) {
    GetFirstAngleAxis(i);
    GetSecondAngleAxis(i); 
    /*If the Axis is unpopulated, it was unable to find two points*/
    vector<float> default_Q;
    default_Q.push_back(1);
    default_Q.push_back(0);
    default_Q.push_back(0);
    default_Q.push_back(0);
    if (AngleAxis1.size() != 2) {
      angle = default_Q;
      return;
    }
    if(OriginalAxis1.magnitude() == 0 || vAngleAxis1.magnitude() == 0) {
      angle = default_Q;
      return;
    }
    vector<float> Q1;
    // Point cross1 = OriginalAxis1.cross(vAngleAxis1);
    Point cross1 = vAngleAxis1.cross(OriginalAxis1).times(.5);
    Q1.push_back(sqrt(pow(vAngleAxis1.magnitude(), 2) * pow(OriginalAxis1.magnitude(), 2)) + vAngleAxis1.dot(OriginalAxis1));
    Q1.push_back(cross1.z);
    Q1.push_back(cross1.y);
    Q1.push_back(cross1.x);
    angle = Qnormalize(Q1);
    // return;
    /*If the Axis is unpopulated, it was unable to find two points*/
    if (AngleAxis2.size() != 2) {
      return;
    }
    if(OriginalAxis2.magnitude() == 0 || vAngleAxis2.magnitude() == 0) {
      return;
    }
    vector<float> Q2;
    Point temp = QRotate(vAngleAxis2, angle);
    Point cross2 = temp.cross(OriginalAxis2).times(.5);
    Q2.push_back(sqrt(pow(temp.magnitude(), 2) * pow(OriginalAxis2.magnitude(), 2)) + temp.dot(OriginalAxis2));
    Q2.push_back(cross2.z);
    Q2.push_back(cross2.y);
    Q2.push_back(cross2.x);
    // angle = Q;
    printf("final\n");
    angle = Qnormalize(Qmult(Q1, Q2));
    return;
  }
  void PrintPoints() {
    vector<Point>::iterator i;
    for (i = points.begin(); i != points.end(); ++i) {
      printf("%s\n",i->print().c_str());
    }
  }
};

}  // namespace object_server
#endif
