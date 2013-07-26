#include <algorithm>
#include <vector>
#include "gtest/gtest.h"
#include "../PhaseSpace/Point.h"
#include "../PhaseSpace/ManualObject.h"
#include "../PhaseSpace/Object.h"
#include "../PhaseSpace/GloveType.h"
#include "../PhaseSpace/DefaultType.h"

using std::vector;
using object_server::Point;
using object_server::Object;
using object_server::ManualObject;
using object_server::GloveType;
using object_server::DefaultType;

class CollidesTest : public ::testing::Test {
  protected :
    ManualObject* CubeL0D1R0;
    ManualObject* CubeL0D2R0;
    ManualObject* Mutable;
    virtual void SetUp() {
      Mutable = new ManualObject;
      CubeL0D1R0 = new ManualObject;
      CubeL0D2R0 = new ManualObject;
      Mutable->init(-1, "mutatable");
      Mutable->SetCenter(0, 0, 0);
      Mutable->SetAngle(1, 0, 0, 0);
      Mutable->SetDim(1, 1, 1);
      CubeL0D1R0->init(0, "cube1");
      CubeL0D1R0->SetCenter(0, 0, 0);
      CubeL0D1R0->SetAngle(1, 0, 0, 0);
      CubeL0D1R0->SetDim(1, 1, 1);
      CubeL0D2R0->init(1, "cube1");
      CubeL0D2R0->SetCenter(0, 0, 0);
      CubeL0D2R0->SetAngle(1, 0, 0, 0);
      CubeL0D2R0->SetDim(2, 2, 2);
    }
    virtual void TearDown() {
      delete CubeL0D1R0;
      delete CubeL0D2R0;
      delete Mutable;
    }  
};

TEST_F(CollidesTest, CollidesWithSelf) {
  ASSERT_TRUE(CubeL0D1R0->CollidesWith(CubeL0D1R0)) << "A surrounds A";
}

TEST_F(CollidesTest, Surround) {
  EXPECT_TRUE(CubeL0D2R0->CollidesWith(CubeL0D1R0)) << "A surrounds B";
  EXPECT_TRUE(CubeL0D1R0->CollidesWith(CubeL0D2R0)) << "B surrounds A";
}

TEST_F(CollidesTest, EdgeCasesNoRotationI) {
  Mutable->SetCenter(1, 0, 0);
  EXPECT_TRUE(CubeL0D1R0->CollidesWith(Mutable)) << "Collide +X";
  Mutable->SetCenter(0, 1, 0);
  EXPECT_TRUE(CubeL0D1R0->CollidesWith(Mutable)) << "Collide +Y";
  Mutable->SetCenter(0, 0, 1);
  EXPECT_TRUE(CubeL0D1R0->CollidesWith(Mutable)) << "Collide +Z";
  Mutable->SetCenter(1, 1, 0);
  EXPECT_TRUE(CubeL0D1R0->CollidesWith(Mutable)) << "Collide +XY";
  Mutable->SetCenter(0, 1, 1);
  EXPECT_TRUE(CubeL0D1R0->CollidesWith(Mutable)) << "Collide +YZ";
  Mutable->SetCenter(1, 0, 1);
  EXPECT_TRUE(CubeL0D1R0->CollidesWith(Mutable)) << "Collide +XZ";
  
  Mutable->SetCenter(1.001, 0, 0);
  EXPECT_FALSE(CubeL0D1R0->CollidesWith(Mutable)) << "!Collide +X";
  Mutable->SetCenter(0, 1.001, 0);
  EXPECT_FALSE(CubeL0D1R0->CollidesWith(Mutable)) << "!Collide +Y";
  Mutable->SetCenter(0, 0, 1.001);
  EXPECT_FALSE(CubeL0D1R0->CollidesWith(Mutable)) << "!Collide +Z";
  Mutable->SetCenter(1.001, 1.001, 0);
  EXPECT_FALSE(CubeL0D1R0->CollidesWith(Mutable)) << "!Collide +XY";
  Mutable->SetCenter(0, 1.001, 1.001);
  EXPECT_FALSE(CubeL0D1R0->CollidesWith(Mutable)) << "!Collide +YZ";
  Mutable->SetCenter(1.001, 0, 1.001);
  EXPECT_FALSE(CubeL0D1R0->CollidesWith(Mutable)) << "!Collide +XZ";
}
TEST_F(CollidesTest, EdgeCasesNoRotationIII) {
  Mutable->SetCenter(-1, 0, 0);
  EXPECT_TRUE(CubeL0D1R0->CollidesWith(Mutable)) << "Collide -X";
  Mutable->SetCenter(0, -1, 0);
  EXPECT_TRUE(CubeL0D1R0->CollidesWith(Mutable)) << "Collide -Y";
  Mutable->SetCenter(0, 0, -1);
  EXPECT_TRUE(CubeL0D1R0->CollidesWith(Mutable)) << "Collide -Z";
  Mutable->SetCenter(-1, -1, 0);
  EXPECT_TRUE(CubeL0D1R0->CollidesWith(Mutable)) << "Collide -XY";
  Mutable->SetCenter(0, -1, -1);
  EXPECT_TRUE(CubeL0D1R0->CollidesWith(Mutable)) << "Collide -YZ";
  Mutable->SetCenter(-1, 0, -1);
  EXPECT_TRUE(CubeL0D1R0->CollidesWith(Mutable)) << "Collide -XZ";

  Mutable->SetCenter(-1.001, 0, 0); 
  EXPECT_FALSE(CubeL0D1R0->CollidesWith(Mutable)) << "!Collide -X";
  Mutable->SetCenter(0, -1.001, 0); 
  EXPECT_FALSE(CubeL0D1R0->CollidesWith(Mutable)) << "!Collide -Y";
  Mutable->SetCenter(0, 0, -1.001);
  EXPECT_FALSE(CubeL0D1R0->CollidesWith(Mutable)) << "!Collide -Z";
  Mutable->SetCenter(-1.001, -1.001, 0); 
  EXPECT_FALSE(CubeL0D1R0->CollidesWith(Mutable)) << "!Collide -XY";
  Mutable->SetCenter(0, -1.001, -1.001);
  EXPECT_FALSE(CubeL0D1R0->CollidesWith(Mutable)) << "!Collide -YZ";
  Mutable->SetCenter(-1.001, 0, -1.001);
  EXPECT_FALSE(CubeL0D1R0->CollidesWith(Mutable)) << "!Collide -XZ";
}
TEST_F(CollidesTest, EdgeCasesNoRotationII_IV) {
  Mutable->SetCenter(-1, 1, 0);
  EXPECT_TRUE(CubeL0D1R0->CollidesWith(Mutable)) << "Collide -X+Y";
  Mutable->SetCenter(0, -1, 1);
  EXPECT_TRUE(CubeL0D1R0->CollidesWith(Mutable)) << "Collide -Y+Z";
  Mutable->SetCenter(-1, 0, 1);
  EXPECT_TRUE(CubeL0D1R0->CollidesWith(Mutable)) << "Collide -X+Z";

  Mutable->SetCenter(1, -1, 0);
  EXPECT_TRUE(CubeL0D1R0->CollidesWith(Mutable)) << "Collide -X+Y";
  Mutable->SetCenter(0, 1, -1);
  EXPECT_TRUE(CubeL0D1R0->CollidesWith(Mutable)) << "Collide Y-Z";
  Mutable->SetCenter(1, 0, -1);
  EXPECT_TRUE(CubeL0D1R0->CollidesWith(Mutable)) << "Collide X-Z";

  Mutable->SetCenter(-1.001, 1.001, 0); 
  EXPECT_FALSE(CubeL0D1R0->CollidesWith(Mutable)) << "!Collide -X+Y";
  Mutable->SetCenter(0, -1.001, 1.001);
  EXPECT_FALSE(CubeL0D1R0->CollidesWith(Mutable)) << "!Collide -Y+Z";
  Mutable->SetCenter(-1.001, 0, 1.001);
  EXPECT_FALSE(CubeL0D1R0->CollidesWith(Mutable)) << "!Collide -X+Z";

  Mutable->SetCenter(1.001, -1.001, 0);
  EXPECT_FALSE(CubeL0D1R0->CollidesWith(Mutable)) << "!Collide -X+Y";
  Mutable->SetCenter(0, 1.001, -1.001);
  EXPECT_FALSE(CubeL0D1R0->CollidesWith(Mutable)) << "!Collide Y-Z";
  Mutable->SetCenter(1.001, 0, -1.001);
  EXPECT_FALSE(CubeL0D1R0->CollidesWith(Mutable)) << "!Collide X-Z";
}
TEST_F(CollidesTest, EdgeCasesOneRotation) {
  // Rotate 45 degress around the z axis
  Mutable->SetCenter(0, 0, 0);
  Mutable->SetAngle(.92388, 0, 0, .92388);
  Mutable->SetCenter(.49 + sqrt(.250), 0, 0);
  EXPECT_TRUE(CubeL0D1R0->CollidesWith(Mutable)) << "Collides Face->Edge";
  Mutable->SetCenter(0, .49 + sqrt(.250), 0);
  EXPECT_TRUE(CubeL0D1R0->CollidesWith(Mutable)) << "Collides Face->Edge";
  Mutable->SetCenter(0, 0, .49 + sqrt(.250));
  EXPECT_TRUE(CubeL0D1R0->CollidesWith(Mutable)) << "Collides Face->Edge";

  Mutable->SetCenter(-.49 - sqrt(.250), 0, 0);
  EXPECT_TRUE(CubeL0D1R0->CollidesWith(Mutable)) << "Collides Face->Edge";
  Mutable->SetCenter(0, -.49 - sqrt(.250), 0);
  EXPECT_TRUE(CubeL0D1R0->CollidesWith(Mutable)) << "Collides Face->Edge";
  Mutable->SetCenter(0, 0, -.49 - sqrt(.250));
  EXPECT_TRUE(CubeL0D1R0->CollidesWith(Mutable)) << "Collides Face->Edge";

  Mutable->SetCenter(.49 + sqrt(.250), .49 + sqrt(.125), 0);
  EXPECT_TRUE(CubeL0D1R0->CollidesWith(Mutable)) << "Collides Edge->Edge";
  Mutable->SetCenter(0, .49 + sqrt(.250), .49 + sqrt(.125));
  EXPECT_TRUE(CubeL0D1R0->CollidesWith(Mutable)) << "Collides Edge->Edge";
  Mutable->SetCenter(.49 + sqrt(.250), 0, .49 + sqrt(.125));
  EXPECT_TRUE(CubeL0D1R0->CollidesWith(Mutable)) << "Collides Edge->Edge";

  Mutable->SetCenter(.5 + sqrt(.250), 0, 0);
  EXPECT_FALSE(CubeL0D1R0->CollidesWith(Mutable)) << "Collides Face->Edge";
  Mutable->SetCenter(0, .5 + sqrt(.250), 0);
  EXPECT_FALSE(CubeL0D1R0->CollidesWith(Mutable)) << "Collides Face->Edge";
  Mutable->SetCenter(0, 0, .5 + sqrt(.250));
  EXPECT_FALSE(CubeL0D1R0->CollidesWith(Mutable)) << "Collides Face->Edge";

  Mutable->SetCenter(-.5 - sqrt(.250), 0, 0);
  EXPECT_FALSE(CubeL0D1R0->CollidesWith(Mutable)) << "Collides Face->Edge";
  Mutable->SetCenter(0, -.5 - sqrt(.250), 0);
  EXPECT_FALSE(CubeL0D1R0->CollidesWith(Mutable)) << "Collides Face->Edge";
  Mutable->SetCenter(0, 0, -.5 - sqrt(.250));
  EXPECT_FALSE(CubeL0D1R0->CollidesWith(Mutable)) << "Collides Face->Edge";

  Mutable->SetCenter(.5 + sqrt(.250), .5 + sqrt(.125), 0);
  EXPECT_FALSE(CubeL0D1R0->CollidesWith(Mutable)) << "Collides Edge->Edge";
  Mutable->SetCenter(0, .5 + sqrt(.250), .5 + sqrt(.125));
  EXPECT_FALSE(CubeL0D1R0->CollidesWith(Mutable)) << "Collides Edge->Edge";
  Mutable->SetCenter(.5 + sqrt(.250), 0, .5 + sqrt(.125));
  EXPECT_FALSE(CubeL0D1R0->CollidesWith(Mutable)) << "Collides Edge->Edge";
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

