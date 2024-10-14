#include <gtest/gtest.h>

#include <Eigen/Core>

#include "TT/transform.h"

constexpr double tolerance = 0.00000000001;

TEST(Instantiation, DefaultConstructor) {
    const tt::Transform xform;

    ASSERT_EQ(Eigen::Vector3d::Zero(), xform.getLocalTranslation());
    ASSERT_EQ(Eigen::Vector3d::Zero(), xform.getLocalRotationEuler());
    ASSERT_EQ(Eigen::Vector3d::Zero(), xform.getWorldTranslation());
    ASSERT_EQ(Eigen::Vector3d::Zero(), xform.getWorldRotationEuler());
    ASSERT_EQ(Eigen::Matrix3d::Identity(), xform.getLocalRotationMatrix());
    ASSERT_EQ(Eigen::Matrix3d::Identity(), xform.getWorldRotationMatrix());
}

TEST(Rotation, SetRotationMatrix) {
    tt::Transform parent(0, 0, 0, 4, 2, 99);
    parent.setLocalRotationMatrix(Eigen::Matrix3d::Identity());
    ASSERT_EQ(Eigen::Matrix3d::Identity(), parent.getLocalRotationMatrix());
}

// WARNING: The angles may be +/- Pi when going to/from matrices. Here we test to ensure that the result is the same.
TEST(Rotation, EulerAngles) {
    const Eigen::Vector3d rotation(24, M_PI_4, -M_PI_4);

    // Parent 1 and 2 should have effectively the same rotation.
    const tt::Transform parent1(Eigen::Vector3d::Zero(), rotation);
    const tt::Transform parent2(Eigen::Vector3d::Zero(), parent1.getLocalRotationEuler());

    tt::Transform child({5, 10, 15}, Eigen::Vector3d::Zero(), &parent1);

    const Eigen::Vector3d childPosParent1 = child.getWorldTranslation();
    child.setParent(&parent2);
    const Eigen::Vector3d childPosParent2 = child.getWorldTranslation();

    ASSERT_NEAR(childPosParent1.x(), childPosParent2.x(), tolerance);
    ASSERT_NEAR(childPosParent1.y(), childPosParent2.y(), tolerance);
    ASSERT_NEAR(childPosParent1.z(), childPosParent2.z(), tolerance);
}

TEST(Parent, SingleParentWithTranslation) {
    const tt::Transform parent(5, 10, 15, 0, 0, 0);
    const tt::Transform child(2, 3, 4, 0, 0, 0, &parent);

    ASSERT_EQ(Eigen::Vector3d(2, 3, 4), child.getLocalTranslation());
    ASSERT_EQ(Eigen::Vector3d(7, 13, 19), child.getWorldTranslation());
}

TEST(Parent, MultiParentWithTranslation) {
    const tt::Transform parent(5, 10, 15, 0, 0, 0);
    const tt::Transform child1(2, 3, 4, 0, 0, 0, &parent);
    const tt::Transform child2(2, 3, 4, 0, 0, 0, &child1);
    const tt::Transform child3(2, 3, 4, 0, 0, 0, &child2);

    ASSERT_EQ(Eigen::Vector3d(2, 3, 4), child3.getLocalTranslation());
    ASSERT_EQ(Eigen::Vector3d(11, 19, 27), child3.getWorldTranslation());
}

TEST(Parent, SingleParentWithTranslationAndPitch) {
    const tt::Transform parent(0, 0, 10, 0, M_PI_2, 0);
    const tt::Transform child(0, 0, 10, 0, 0, 0, &parent);

    const auto worldTranslation = child.getWorldTranslation();

    ASSERT_EQ(Eigen::Vector3d(0, 0, 10), child.getLocalTranslation());
    ASSERT_NEAR(10, worldTranslation.x(), tolerance);
    ASSERT_NEAR(0, worldTranslation.y(), tolerance);
    ASSERT_NEAR(10, worldTranslation.z(), tolerance);
}

TEST(Translation, SetLocalTranslation) {
    tt::Transform parent(0, 0, 10, 0, 0, 0);
    parent.setLocalTranslation({2, 5, 9});
    ASSERT_EQ(Eigen::Vector3d(2, 5, 9), parent.getLocalTranslation());
}
