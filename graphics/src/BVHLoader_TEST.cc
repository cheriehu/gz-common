/*
 * Copyright (C) 2022 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <gtest/gtest.h>

#include "gz/common/BVHLoader.hh"
#include <gz/common/AssimpLoader.hh>
#include <gz/common/Mesh.hh>
#include "gz/common/Skeleton.hh"
#include "gz/common/SkeletonAnimation.hh"
#include "gz/common/testing/AutoLogFixture.hh"
#include "gz/common/testing/TestPaths.hh"

using namespace gz;

class BHVLoaderTest : public common::testing::AutoLogFixture { };

void printNode(common::SkeletonNode* node, int depth)
{
    if (!node) return;
    std::cout << std::string(depth * 2, ' ') << "Name: " << node->Name() << ", Id: " << node->Id() << std::endl;
    std::cout << std::string(depth * 2, ' ') << "Transform:" << node->Transform() << std::endl;
    for (unsigned int i = 0; i < node->ChildCount(); ++i) {
      printNode(node->Child(i), depth + 1);
    }
}

/////////////////////////////////////////////////
TEST_F(BHVLoaderTest, LoadBVH)
{
  common::BVHLoader loader;
  auto skel = loader.Load("", 1);
  EXPECT_EQ(nullptr, skel);

  skel = loader.Load(
      common::testing::TestFile("data", "cmu-13_26.bvh"), 1);
  EXPECT_NE(nullptr, skel->RootNode());

  EXPECT_EQ(skel->RootNode()->Name(), std::string("Hips"));
  EXPECT_EQ(31u, skel->NodeCount());
  std::cout << "--- SKELETON HIERARCHY ---" << std::endl;
  printNode(skel->RootNode(), 0);
  std::cout << "--------------------------" << std::endl;
}

TEST_F(BHVLoaderTest, LoadBVHAssimp)
{
  gz::common::AssimpLoader loader;
  std::unique_ptr<gz::common::Mesh> mesh(loader.Load(
      common::testing::TestFile("data", "cmu-13_26.bvh")));
  ASSERT_NE(nullptr, mesh);

  auto skel = mesh->MeshSkeleton();
  ASSERT_NE(nullptr, skel);

  ASSERT_NE(0u, skel->AnimationCount());

  skel->Scale(1);
  EXPECT_NE(nullptr, skel->RootNode());

  EXPECT_EQ(skel->RootNode()->Name(), std::string("Hips"));
  EXPECT_EQ(31u, skel->NodeCount());
  printNode(skel->RootNode(), 0);

  common::BVHLoader bvhLoader;
  auto bvhSkel = bvhLoader.Load(common::testing::TestFile("data", "cmu-13_26.bvh"), 1);
  ASSERT_NE(nullptr, bvhSkel);
  auto bvhAnim = bvhSkel->Animation(0);
  auto assimpAnim = skel->Animation(0);

  EXPECT_NEAR(bvhAnim->Length(), assimpAnim->Length(), 1e-4);
  EXPECT_EQ(bvhAnim->NodeCount(), assimpAnim->NodeCount());

  for (const std::string joint : {"Hips", "LeftUpLeg", "RightArm"})
  {
    EXPECT_EQ(bvhSkel->NodeByName(joint)->Transform(), skel->NodeByName(joint)->Transform());
    EXPECT_TRUE(bvhAnim->NodePoseAt(joint, 0.0).Equal(assimpAnim->NodePoseAt(joint, 0.0), 1e-4));
    EXPECT_TRUE(bvhAnim->NodePoseAt(joint, 1.0).Equal(assimpAnim->NodePoseAt(joint, 1.0), 1e-4));
  }
}
