#include <algorithm>
#include <vector>
#include "gtest/gtest.h"
#include "../PhaseSpace/Point.h"
#include "../PhaseSpace/ManualObject.h"
#include "../PhaseSpace/PSObject.h"
#include "../PhaseSpace/Object.h"
#include "../PhaseSpace/GloveType.h"
#include "../PhaseSpace/DefaultType.h"

using object_server::ManualObject;
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

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

