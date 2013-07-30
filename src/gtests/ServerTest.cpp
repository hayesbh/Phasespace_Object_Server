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
      p.id_ = i;
      p.current_ = 1;
      p.Init(i, j, k);
      points.push_back(p);
      i++; j--; k++;
    }
    default_object->Init(0, "default1", points, "default", false);
    //glove_object->Init(1, "glove1", points, "glove", 0);
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
  printf("Here\n");
  Object* revived;
  EXPECT_TRUE(revive_object("obj", ids_set, &revived, 0));
  EXPECT_TRUE(revived->get_center().Equals(obj->get_center()));
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
  printf("Here\n");
  Object* revive;
  EXPECT_TRUE(revive_object("default1", ids_set, &revive, 0));
  PSObject* revived = static_cast<PSObject*>(revive);
  EXPECT_TRUE(revived->get_center().Equals(obj->get_center()));
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
  EXPECT_TRUE(revived->get_pointer().Equals(obj->get_pointer()));
  EXPECT_TRUE(revived->get_axis1_ids()[0] == obj->get_axis1_ids()[0] &&
              revived->get_axis1_ids()[1] == obj->get_axis1_ids()[1]);
  EXPECT_TRUE(revived->get_original_axis1().Equals(obj->get_original_axis1()));
  EXPECT_TRUE(revived->get_axis2_ids()[0] == obj->get_axis2_ids()[0] &&
              revived->get_axis2_ids()[1] == obj->get_axis2_ids()[1]);
  EXPECT_TRUE(revived->get_original_axis2().Equals(obj->get_original_axis2()));
  EXPECT_TRUE(fabs(revived->get_rotation()[0] - obj->get_rotation()[0]) < .001 && fabs(revived->get_rotation()[1] - obj->get_rotation()[1]) < .001 &&
              fabs(revived->get_rotation()[2] - obj->get_rotation()[2]) < .001 && fabs(revived->get_rotation()[3] - obj->get_rotation()[3]) < .001);
  EXPECT_TRUE(revived->get_axes()[0].Equals(obj->get_axes()[0]) && revived->get_axes()[1].Equals(obj->get_axes()[1]) && revived->get_axes()[2].Equals(obj->get_axes()[2]));
  EXPECT_TRUE(fabs(revived->get_dimensions()[0] - obj->get_dimensions()[0]) < .001 && fabs(revived->get_dimensions()[1] - obj->get_dimensions()[1]) < .001 &&
              fabs(revived->get_dimensions()[2] - obj->get_dimensions()[2]) < .001);
  EXPECT_TRUE(revived->get_rigidity() == obj->get_rigidity());
}



int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

