#include "TT/transform.h"

tt::Transform::Transform() :
  xform_(Eigen::Matrix4d::Identity()),
  parent_(nullptr) {
}

tt::Transform::Transform(const double x, const double y, const double z, const double roll, const double pitch,
                         const double yaw, const Transform* parent) :
  parent_(parent) {
  xform_ = Eigen::Translation3d(x, y, z) *
    Eigen::Matrix3d(
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
}

tt::Transform::Transform(const Eigen::Vector3d& translation, const Eigen::Vector3d& rotation, const Transform* parent) :
  Transform(
    translation.x(), translation.y(), translation.z(), rotation.x(), rotation.y(), rotation.z(), parent) {
}

tt::Transform::Transform(const Eigen::Transform<double, 3, Eigen::Affine>& xform, const Transform* parent) :
  xform_(xform),
  parent_(parent) {
}

tt::Transform tt::Transform::toLocalTransform(const Transform& worldSpaceTransform) const {
  return Transform(xform_.inverse() * worldSpaceTransform.xform_);
}

Eigen::Vector3d tt::Transform::toLocalVector(const Eigen::Vector3d& worldSpaceVector) const {
  const Eigen::Vector4d inVector{worldSpaceVector.x(), worldSpaceVector.y(), worldSpaceVector.z(), 0};
  return (xform_.inverse() * inVector).block<3, 1>(0, 0);
}

tt::Transform tt::Transform::toWorldTransform() const {
  if (parent_ == nullptr) {
    return *this;
  }
  return Transform(parent_->toWorldTransform().xform_ * xform_);
}

Eigen::Vector3d tt::Transform::getLocalTranslation() const {
  return xform_.translation();
}

void tt::Transform::setParent(const Transform* parent) {
  this->parent_ = parent;
}

const tt::Transform* tt::Transform::getParent() const {
  return parent_;
}

Eigen::Transform<double, 3, Eigen::Affine>& tt::Transform::data() {
  return xform_;
}

void tt::Transform::setLocalTranslation(const Eigen::Vector3d& translation) {
  xform_.translation() = translation;
}

Eigen::Matrix3d tt::Transform::getLocalRotationMatrix() const {
  return xform_.rotation();
}

Eigen::Vector3d tt::Transform::getLocalRotationEuler() const {
  return xform_.rotation().eulerAngles(2, 1, 0).reverse();
}

void tt::Transform::setLocalRotationMatrix(const Eigen::Matrix3d& rotation) {
  xform_.matrix().block<3, 3>(0, 0) = rotation;
}

void tt::Transform::setLocalRotationEuler(const Eigen::Vector3d& rotation) {
  xform_.matrix().block<3, 3>(0, 0) = Eigen::Matrix3d(
    Eigen::AngleAxisd(rotation.z(), Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(rotation.y(), Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(rotation.x(), Eigen::Vector3d::UnitX()));
}

Eigen::Vector3d tt::Transform::getWorldTranslation() const {
  return toWorldTransform().getLocalTranslation();
}

Eigen::Matrix3d tt::Transform::getWorldRotationMatrix() const {
  return toWorldTransform().getLocalRotationMatrix();
}

Eigen::Vector3d tt::Transform::getWorldRotationEuler() const {
  return toWorldTransform().getLocalRotationEuler();
}
