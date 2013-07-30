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
  PSObject* ps_object;
  virtual void SetUp() {
    manual_object = new ManualObject;
    ps_object = new PSObject;
  }
  virtual ~StoreObject() {
    delete manual_object;
    delete ps_object;
  }
};

TEST_F(StoreObject, ManualObject) {
  vector<int> ids_set;
  ManualObject* Cube = new ManualObject;
  Cube->Init(0, "cube1");
  Cube->SetCenter(0, 0, 0);
  Cube->SetAngle(1, 0, 0, 0);
  Cube->SetDim(1, 1, 1);
  Object* obj = static_cast<Object*>(Cube);
  store_object(obj, "obj");
  return;
  Object* revived;
  revive_object("obj", ids_set, &revived, 0);
  EXPECT_TRUE(revived->get_center().Equals(obj->get_center()));
  EXPECT_TRUE(revived->get_type() == obj->get_type());
  EXPECT_TRUE(revived->get_pointer().Equals(obj->get_pointer()));
  EXPECT_TRUE(fabs(revived->get_rotation()[0] - obj->get_rotation()[0]) < .001 && fabs(revived->get_rotation()[1] - obj->get_rotation()[1]) < .001 &&
              fabs(revived->get_rotation()[2] - obj->get_rotation()[2]) < .001 && fabs(revived->get_rotation()[3] - obj->get_rotation()[3]) < .001);
  EXPECT_TRUE(revived->get_axes()[0].Equals(obj->get_axes()[0]) && revived->get_axes()[1].Equals(obj->get_axes()[1]) && revived->get_axes()[2].Equals(obj->get_axes()[2]));
  EXPECT_TRUE(fabs(revived->get_dimensions()[0] - obj->get_rotation()[0]) < .001 && fabs(revived->get_dimensions()[1] - obj->get_rotation()[1]) < .001 &&
              fabs(revived->get_dimensions()[2] - obj->get_rotation()[2]) < .001);
  EXPECT_TRUE(revived->get_rigidity() == obj->get_rigidity());
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

