#include <algorithm>
#include <vector>
#include "gtest/gtest.h"
#include "../PhaseSpace/Point.h"
#include "../PhaseSpace/ManualObject.h"
#include "../PhaseSpace/PSObject.h"
#include "../PhaseSpace/Object.h"
#include "../PhaseSpace/GloveType.h"
#include "../PhaseSpace/DefaultType.h"
#include "../save.h"

using object_server::revive_object;
using object_server::store_object;
using object_server::store_env;
using object_server::restore_env;
using object_server::ManualObject;
using object_server::Object;
using object_server::PSObject;
using object_server::Point;
using object_server::PSObject;
using std::vector;

class StoreObject : public ::testing::Test {
  protected:
  ManualObject* manual_object;
  PSObject* default_object;
  PSObject* glove_object;
  vector<Point> points;
  virtual void SetUp() {
    manual_object = new ManualObject;
    default_object = new PSObject;
    glove_object = new PSObject;
    int i = 0; int j = 6; int k = 12;
    while(i != 7) {
      Point p;
      p.Init(i, j, k);
      p.id_ = i;
      p.current_ = 1;
      points.push_back(p);
      i++; j--; k++;
    }
    default_object->Init(0, "default1", points, "default", true);
    glove_object->Init(1, "glove1", points, "glove", true);
  }
  virtual ~StoreObject() {
    delete manual_object;
    delete default_object;
    delete glove_object;
  }
};

TEST_F(StoreObject, ManualObject) {
  vector<int> ids_set;
  manual_object->Init(0, "cube1");
  manual_object->SetCenter(0, 0, 0);
  manual_object->SetAngle(1, 0, 0, 0);
  manual_object->SetDim(1, 1, 1);
  Object* obj = static_cast<Object*>(manual_object);
  EXPECT_TRUE(store_object(obj, "obj"));
  Object* revived;
  EXPECT_TRUE(revive_object("obj", ids_set, &revived, 0));
  EXPECT_TRUE(revived->get_center().Equals(obj->get_center())) << revived->get_center() << obj->get_center();
  EXPECT_TRUE(revived->get_type() == obj->get_type());
  EXPECT_TRUE(revived->get_pointer().Equals(obj->get_pointer()));
  EXPECT_TRUE(fabs(revived->get_rotation()[0] - obj->get_rotation()[0]) < .001 && fabs(revived->get_rotation()[1] - obj->get_rotation()[1]) < .001 &&
              fabs(revived->get_rotation()[2] - obj->get_rotation()[2]) < .001 && fabs(revived->get_rotation()[3] - obj->get_rotation()[3]) < .001);
  EXPECT_TRUE(revived->get_axes()[0].Equals(obj->get_axes()[0]) && revived->get_axes()[1].Equals(obj->get_axes()[1]) && revived->get_axes()[2].Equals(obj->get_axes()[2]));
  EXPECT_TRUE(fabs(revived->get_dimensions()[0] - obj->get_dimensions()[0]) < .001 && fabs(revived->get_dimensions()[1] - obj->get_dimensions()[1]) < .001 &&
              fabs(revived->get_dimensions()[2] - obj->get_dimensions()[2]) < .001);
  EXPECT_TRUE(revived->get_rigidity() == obj->get_rigidity());
}

TEST_F(StoreObject, DefaultObject) {
  vector<int> ids_set;
  Object* object = static_cast<Object*>(default_object);
  EXPECT_TRUE(store_object(default_object, "default1"));
  PSObject* obj = static_cast<PSObject*>(object);
  Object* revive;
  EXPECT_TRUE(revive_object("default1", ids_set, &revive, 0));
  EXPECT_FALSE(revive_object("default1", ids_set, &revive, 0));
  PSObject* revived = static_cast<PSObject*>(revive);
  vector<Point>::iterator iter1;
  vector<Point>::iterator iter2;
  vector<Point> point_set1 = obj->get_points();
  vector<Point> point_set2 = revived->get_points();
  iter1 = point_set1.begin();
  iter2 = point_set2.begin();
  while(iter1 != point_set1.end() && iter2 != point_set2.end()) {
    EXPECT_TRUE(iter1->id_ == iter2->id_);
    ++iter1; ++iter2;
  }
  EXPECT_TRUE(revived->get_type() == obj->get_type());
  EXPECT_TRUE(revived->get_pointer().id_ == obj->get_pointer().id_);
  EXPECT_TRUE(revived->get_axis1_ids()[0] == obj->get_axis1_ids()[0] &&
              revived->get_axis1_ids()[1] == obj->get_axis1_ids()[1]);
  EXPECT_TRUE(revived->get_original_axis1().Equals(obj->get_original_axis1()));
  EXPECT_EQ(revived->get_axis2_ids()[0] , obj->get_axis2_ids()[0]) << revived->get_axis2_ids()[0] << ", " << obj->get_axis2_ids()[0];
  EXPECT_EQ(revived->get_axis2_ids()[1] , obj->get_axis2_ids()[1]) << revived->get_axis2_ids()[0] << ", " << obj->get_axis2_ids()[0];
  EXPECT_TRUE(revived->get_original_axis2().Equals(obj->get_original_axis2())) << revived->get_original_axis2() << obj->get_original_axis2();
  EXPECT_TRUE(fabs(revived->get_dimensions()[0] - obj->get_dimensions()[0]) < .001 && fabs(revived->get_dimensions()[1] - obj->get_dimensions()[1]) < .001 &&
              fabs(revived->get_dimensions()[2] - obj->get_dimensions()[2]) < .001);
  EXPECT_TRUE(revived->get_rigidity() == obj->get_rigidity());
}

TEST_F(StoreObject, GloveObject) {
  vector<int> ids_set;
  Object* object = static_cast<Object*>(glove_object);
  EXPECT_TRUE(store_object(glove_object, "glove1"));
  PSObject* obj = static_cast<PSObject*>(object);
  Object* revive;
  EXPECT_TRUE(revive_object("glove1", ids_set, &revive, 0));
  EXPECT_FALSE(revive_object("glove1", ids_set, &revive, 1));
  PSObject* revived = static_cast<PSObject*>(revive);
  vector<Point>::iterator iter1;
  vector<Point>::iterator iter2;
  vector<Point> point_set1 = obj->get_points();
  vector<Point> point_set2 = revived->get_points();
  iter1 = point_set1.begin();
  iter2 = point_set2.begin();
  while(iter1 != point_set1.end() && iter2 != point_set2.end()) {
    EXPECT_TRUE(iter1->id_ == iter2->id_);
    ++iter1; ++iter2;
  }
  EXPECT_EQ(revived->get_type() , obj->get_type());
  EXPECT_TRUE(revived->get_pointer().id_ == obj->get_pointer().id_);
  EXPECT_TRUE(revived->get_axis1_ids()[0] == obj->get_axis1_ids()[0] &&
              revived->get_axis1_ids()[1] == obj->get_axis1_ids()[1]);
  EXPECT_TRUE(revived->get_original_axis1().Equals(obj->get_original_axis1()));
  EXPECT_EQ(revived->get_axis2_ids()[0] , obj->get_axis2_ids()[0]) << revived->get_axis2_ids()[0] << ", " << obj->get_axis2_ids()[0];
  EXPECT_EQ(revived->get_axis2_ids()[1] , obj->get_axis2_ids()[1]) << revived->get_axis2_ids()[0] << ", " << obj->get_axis2_ids()[0];
  EXPECT_TRUE(revived->get_original_axis2().Equals(obj->get_original_axis2())) << revived->get_original_axis2() << obj->get_original_axis2();
  EXPECT_TRUE(fabs(revived->get_dimensions()[0] - obj->get_dimensions()[0]) < .001 && fabs(revived->get_dimensions()[1] - obj->get_dimensions()[1]) < .001 &&
              fabs(revived->get_dimensions()[2] - obj->get_dimensions()[2]) < .001);
  EXPECT_TRUE(revived->get_rigidity() == obj->get_rigidity());
}

class SaveEnv : public ::testing::Test {
  protected:
  ManualObject* manual_object;
  PSObject* default_object;
  PSObject* glove_object;
  vector<Object*> object_vector_;
  vector<int> ids_set_;
  virtual void SetUp() {
    manual_object = new ManualObject;
    default_object = new PSObject;
    glove_object = new PSObject;
    vector<Point> default_points;
    int i = 0; int j = 6; int k = 12;
    while(i != 7) {
      Point p;
      p.Init(i, j, k);
      p.id_ = i;
      p.current_ = 1;
      default_points.push_back(p);
      i++; j--; k++;
    }
    vector<Point> glove_points;
    while(i != 14) {
      Point p;
      p.Init(i, j, k);
      p.id_ = i;
      p.current_ = 1;
      glove_points.push_back(p);
      i++; j--; k++;
    }
    manual_object->Init(0, "cube1");
    manual_object->SetCenter(0, 0, 0);
    manual_object->SetAngle(1, 0, 0, 0);
    manual_object->SetDim(1, 1, 1);
    default_object->Init(1, "default1", default_points, "default", true);
    glove_object->Init(2, "glove1", glove_points, "glove", true);
    Object* man = static_cast<Object*>(manual_object);
    Object* def = static_cast<Object*>(default_object);
    Object* glo = static_cast<Object*>(glove_object);
    object_vector_.push_back(man);
    object_vector_.push_back(def);
    object_vector_.push_back(glo);
  }
  virtual ~SaveEnv() {
    delete manual_object;
    delete default_object;
    delete glove_object;
  }
};

TEST_F(SaveEnv, SaveAndRecover) {
  EXPECT_TRUE(store_env("environments", "SaveEnvEnv", object_vector_));
  vector<Object*> new_object_vector;
  vector<int> ids_set;
  int object_count = 0;
  EXPECT_TRUE(restore_env("environments", "SaveEnvEnv", ids_set, new_object_vector, object_count));
  Object* obj = static_cast<Object*>(new_object_vector[0]);
  EXPECT_TRUE(store_object(obj, "obj"));
  Object* rev;
  EXPECT_TRUE(revive_object("obj", ids_set, &rev, 0));
  EXPECT_TRUE(rev->get_center().Equals(obj->get_center())) << rev->get_center() << obj->get_center();
  EXPECT_TRUE(rev->get_type() == obj->get_type());
  EXPECT_TRUE(rev->get_pointer().Equals(obj->get_pointer()));
  EXPECT_TRUE(fabs(rev->get_rotation()[0] - obj->get_rotation()[0]) < .001 && fabs(rev->get_rotation()[1] - obj->get_rotation()[1]) < .001 &&
              fabs(rev->get_rotation()[2] - obj->get_rotation()[2]) < .001 && fabs(rev->get_rotation()[3] - obj->get_rotation()[3]) < .001);
  EXPECT_TRUE(rev->get_axes()[0].Equals(obj->get_axes()[0]) && rev->get_axes()[1].Equals(obj->get_axes()[1]) && rev->get_axes()[2].Equals(obj->get_axes()[2]));
  EXPECT_TRUE(fabs(rev->get_dimensions()[0] - obj->get_dimensions()[0]) < .001 && fabs(rev->get_dimensions()[1] - obj->get_dimensions()[1]) < .001 &&
              fabs(rev->get_dimensions()[2] - obj->get_dimensions()[2]) < .001);
  EXPECT_TRUE(rev->get_rigidity() == obj->get_rigidity());
  PSObject* revived = static_cast<PSObject*>(new_object_vector[1]);
  vector<Point>::iterator iter1;
  vector<Point>::iterator iter2;
  vector<Point> point_set1 = default_object->get_points();
  vector<Point> point_set2 = revived->get_points();
  iter1 = point_set1.begin();
  iter2 = point_set2.begin();
  while(iter1 != point_set1.end() && iter2 != point_set2.end()) {
    EXPECT_TRUE(iter1->id_ == iter2->id_);
    ++iter1; ++iter2;
  }
  EXPECT_TRUE(revived->get_type() == default_object->get_type());
  EXPECT_TRUE(revived->get_pointer().id_ == default_object->get_pointer().id_);
  EXPECT_TRUE(revived->get_axis1_ids()[0] == default_object->get_axis1_ids()[0] &&
              revived->get_axis1_ids()[1] == default_object->get_axis1_ids()[1]);
  EXPECT_TRUE(revived->get_original_axis1().Equals(default_object->get_original_axis1()));
  EXPECT_EQ(revived->get_axis2_ids()[0] , default_object->get_axis2_ids()[0]) << revived->get_axis2_ids()[0] << ", " << default_object->get_axis2_ids()[0];
  EXPECT_EQ(revived->get_axis2_ids()[1] , default_object->get_axis2_ids()[1]) << revived->get_axis2_ids()[0] << ", " << default_object->get_axis2_ids()[0];
  EXPECT_TRUE(revived->get_original_axis2().Equals(default_object->get_original_axis2())) << revived->get_original_axis2() << default_object->get_original_axis2();
  EXPECT_TRUE(fabs(revived->get_dimensions()[0] - default_object->get_dimensions()[0]) < .001 && fabs(revived->get_dimensions()[1] - default_object->get_dimensions()[1]) < .001 &&
              fabs(revived->get_dimensions()[2] - default_object->get_dimensions()[2]) < .001);
  EXPECT_TRUE(revived->get_rigidity() == default_object->get_rigidity());
  revived = static_cast<PSObject*>(new_object_vector[2]);
  vector<Point> point_set3 = glove_object->get_points();
  vector<Point> point_set4 = revived->get_points();
  iter1 = point_set3.begin();
  iter2 = point_set4.begin();
  while(iter1 != point_set3.end() && iter2 != point_set4.end()) {
    EXPECT_TRUE(iter1->id_ == iter2->id_);
    ++iter1; ++iter2;
  }
  EXPECT_EQ(revived->get_type() , glove_object->get_type());
  EXPECT_TRUE(revived->get_pointer().id_ == glove_object->get_pointer().id_);
  EXPECT_TRUE(revived->get_axis1_ids()[0] == glove_object->get_axis1_ids()[0] &&
              revived->get_axis1_ids()[1] == glove_object->get_axis1_ids()[1]);
  EXPECT_TRUE(revived->get_original_axis1().Equals(glove_object->get_original_axis1()));
  EXPECT_EQ(revived->get_axis2_ids()[0] , glove_object->get_axis2_ids()[0]) << revived->get_axis2_ids()[0] << ", " << glove_object->get_axis2_ids()[0];
  EXPECT_EQ(revived->get_axis2_ids()[1] , glove_object->get_axis2_ids()[1]) << revived->get_axis2_ids()[0] << ", " << glove_object->get_axis2_ids()[0];
  EXPECT_TRUE(revived->get_original_axis2().Equals(glove_object->get_original_axis2())) << revived->get_original_axis2() << glove_object->get_original_axis2();
  EXPECT_TRUE(fabs(revived->get_dimensions()[0] - glove_object->get_dimensions()[0]) < .001 && fabs(revived->get_dimensions()[1] - glove_object->get_dimensions()[1]) < .001 &&
              fabs(revived->get_dimensions()[2] - glove_object->get_dimensions()[2]) < .001);
  EXPECT_TRUE(revived->get_rigidity() == glove_object->get_rigidity());

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

