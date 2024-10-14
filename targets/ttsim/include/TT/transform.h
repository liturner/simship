#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace tt {
  /// \brief A 4x4 Homogeneous Transformation Matrix.
  ///
  /// This class provides a set of usefull functions for transformations in a 3d world. It aims to simplify the usage of
  /// an Eigen::Transform lowering the initial learning curve for linear transformations and exposing an API focused on
  /// simulation use cases.
  ///
  /// This Transform class is aimed at the HLA and DIS standards, and uses the Axis and Rotation described in those
  /// standards. This coordinate system is also often refered to as ECEF, or WorldPosition. While this does not make
  /// much difference mathematically, we refer to x, y, and z axis rotation as roll, pitch and yaw to help improve
  /// understanding. That said:
  ///
  /// - X = Forward
  /// - Y = Left
  /// - Z = Up
  /// - Roll = Rotate Right (X)
  /// - Pitch = Rotate Down (Y)
  /// - Yaw = Rotate Left (Z)
  ///
  /// The decription of this can be found in the DIS standard, but in many cases it is easier to consider your entity as
  /// being "upside down" when it has no rotation applied. The reason for this space being used in simulation, is that
  /// from an entity (e.g. Aircraft) perspective, Roll, Pitch and Yaw become Rotate Right, Up and Right respectively.
  ///
  /// Do not be confused by the North East Down (NED) coordinate system! It is the same coordinate system as the ECEF
  /// coordinate system, only with additional rotations applied. Avoid using it, in most casses it is better to ignore
  /// the concept of Geography completely and work purely in the ECEF coordinate system. NED is generally only needed
  /// for e.g. representing velocity, rotation etc. relative to the planet at exactly your current location. Even
  /// physics effects such as gravity can be better applied without imposing man made concepts onto the pure math.
  class Transform {
  public:
    /// Constructs a Transform at {0, 0, 0} with no rotation and no parent (a 4 x 4 Identity matrix).
    Transform();

    Transform(double x, double y, double z, double roll, double pitch, double yaw,
              const Transform* parent = nullptr);

    Transform(const Eigen::Vector3d& translation, const Eigen::Vector3d& rotation, const Transform* parent = nullptr);

    explicit Transform(const Eigen::Transform<double, 3, Eigen::Affine>& xform, const Transform* parent = nullptr);

    /// Transforms the supplied World Space Transformation to a Transform relative to this one.
    ///
    /// For example, If this is a Radar, and the supplied attribute is the XForm of an enemy entity, this function will
    /// return a Transform that is the offset from the Radar.
    ///
    /// \param worldSpaceTransform
    /// \return
    [[nodiscard]] Transform toLocalTransform(const Transform& worldSpaceTransform) const;

    /// Transforms the supplied world space direction vector to local space.
    ///
    /// This is particularly usefull if you need to e.g. calculate the relative velocity of another entity to yourself.
    /// Consider a Missile Velocity in world space. If you use this function, you will have a Velocity Vector relative
    /// to yourself. In a Radar Echo, the X component would then represent the speed of the object moving away from you.
    /// A simple addition of said vector to your local velocity vector will get the relative velocity.
    ///
    /// Internally this will create a Vector4d, with the w component as 0;
    ///
    /// \param worldSpaceVector
    /// \return
    [[nodiscard]] Eigen::Vector3d toLocalVector(const Eigen::Vector3d& worldSpaceVector) const;

    /// Transforms the supplied world space position vector to local space.
    ///
    /// Internally this will create a Vector4d, with the w component as 1;
    ///
    /// \param worldSpacePosition
    /// \return
    [[nodiscard]] Eigen::Vector3d toLocalPosition(const Eigen::Vector3d& worldSpacePosition) const;

    /// Gets the translation element of this transform, irrelevant of the Transform heierachy. That is to say, the
    /// translation from the world position of the parent transform.
    ///
    /// \return the translation component of this transform
    [[nodiscard]] Eigen::Vector3d getLocalTranslation() const;

    /// Gets the rotation emelemnt of this transform. This method is not effected by the presence of a parent.
    ///
    /// \return the rotation matrix of this transform (upper left 3x3 matrix)
    [[nodiscard]] Eigen::Matrix3d getLocalRotationMatrix() const;

    /// Calculates the Euler rotation angles from the local rotation matrix. Note that this is a mathematically intense
    /// operation which should be avoided if possible. Note that the angles returned may be + / - M_PI compared to those
    /// set previously.
    ///
    /// \return roll, pitch and yaw (respectively) in radians
    [[nodiscard]] Eigen::Vector3d getLocalRotationEuler() const;

    /// Sets the translation component of this Transform. This is most often the position of an entity, or an offset
    /// of a subcomponent to an entity.
    ///
    /// \param translation the x, y, z translation component to use
    void setLocalTranslation(const Eigen::Vector3d& translation);

    /// Sets the rotation component of this Transform. This is most often the rotation of an entitty. This function
    /// sets the upper left 3x3 area to that of the supplied rotation.
    ///
    /// \see setLocalRotationEuler
    ///
    /// \param rotation the 3x3 rotation component to use in this Transform.
    void setLocalRotationMatrix(const Eigen::Matrix3d& rotation);

    /// Sets the rotation component of this Transform. This is most often the rotation of an entitty. This function
    /// sets the upper left 3x3 area to that of the supplied rotation.
    ///
    /// \see setLocalRotationMatrix
    ///
    /// \param rotation
    void setLocalRotationEuler(const Eigen::Vector3d& rotation);

    /// Calculates and returns a single Transform with no parents which represents the complete Transform from World
    /// to Local space. This can also be considered "flattening" the Transform heierachy.
    ///
    /// \return a Transform with no parent
    [[nodiscard]] Transform toWorldTransform() const;

    [[nodiscard]] Transform toWorldTransform(const Transform& localSpaceTransform) const;

    [[nodiscard]] Eigen::Vector3d toWorldVector(const Eigen::Vector3d& localSpaceVector) const;

    [[nodiscard]] Eigen::Vector3d toWorldPosition(const Eigen::Vector3d& localSpacePosition) const;

    /// Calculates and returns the world space position. For example, if this transform has a local transform of
    /// {5, 0, 0}, but is child to a Transform with an offset {0, 10, 0}, then this would return {5, 10, 0} (assuming
    /// no rotation). This is usefull for example when you need to know the world position of a sub system in relation
    /// to the aircraft (e.g. the Radar position in the world, when it is offset forward of the Aircraft position).
    ///
    /// Equivelant to toWorldTransform().getLocalTranslation();
    ///
    /// \see toWorldTransform()
    /// \see getWorldRotationMatrix()
    ///
    /// \return the "world space" position
    [[nodiscard]] Eigen::Vector3d getWorldTranslation() const;

    /// Calculates and returns the world space rotation. For example, if this transform has a local rotation of
    /// {5, 0, 0}, but is child to a Transform with a rotation {0, 10, 0}, then this would return {5, 10, 0}. This is
    /// usefull for example when you need to know the world orientation of a sub system in relation to the aircraft
    /// (e.g. the Radar orientation in the world, when it is offset forward of the Aircraft position).
    ///
    /// As a further example, if a unit vector {1, 0, 0} (forward) is multiplied by this matrix, then the result is a
    /// vector pointing the direction of the Radar beam.
    ///
    /// Equivelant to toWorldTransform().getLocalRotationMatrix();
    ///
    /// \see toWorldTransform()
    /// \see getWorldTranslation()
    ///
    /// \return the "world space" rotation
    [[nodiscard]] Eigen::Matrix3d getWorldRotationMatrix() const;

    /// Calculates and returns the world space rotation in radians, as Euler Angles. Note, that due to how matrices
    /// work, it is not possible to guarentee that the values returned by this function will match those previously
    /// supplied in setters. In particular, several of the parameters may be returned with + / - M_PI
    ///
    /// \return thw "world space" rotation in roll, pitch, yaw.
    [[nodiscard]] Eigen::Vector3d getWorldRotationEuler() const;

    void setParent(const Transform* parent);

    [[nodiscard]] const Transform* getParent() const;

    /// Gets the underlying transform in its raw Eigen format.
    ///
    /// \return a reference to the underlying transform
    Eigen::Transform<double, 3, Eigen::Affine>& data();

  private:
    Eigen::Transform<double, 3, Eigen::Affine> xform_;

    const Transform* parent_;
  };
}
