#pragma once
#include <cstdint>

namespace tt::rpr_fom {
  enum class DeadReckoningAlgorithmEnum8 : uint8_t {
    Other = 0,
    Static = 1,
    DRM_FPW = 2,
    DRM_RPW = 3,
    DRM_RVW = 4,
    DRM_FVW = 5,
    DRM_FPB = 6,
    DRM_RPB = 7,
    DRM_RVB = 8,
    DRM_FVB = 9
  };

  struct OrientationStruct {
    float Psi = 0;

    float Theta = 0;

    float Phi = 0;
  };

  struct WorldLocationStruct {
    double X = 0;

    double Y = 0;

    double Z = 0;
  };

  struct VelocityVectorStruct {
    float XVelocity = 0;

    float YVelocity = 0;

    float ZVelocity = 0;
  };

  struct AccelerationVectorStruct {
    float XAcceleration = 0;

    float YAcceleration = 0;

    float ZAcceleration = 0;
  };

  struct AngularVelocityVectorStruct {
    float XAngularVelocity = 0;

    float YAngularVelocity = 0;

    float ZAngularVelocity = 0;
  };

  struct SpatialRVStruct {
    WorldLocationStruct WorldLocation;

    bool IsFrozen = false;

    OrientationStruct Orientation;

    VelocityVectorStruct VelocityVector;

    AccelerationVectorStruct AccelerationVector;

    AngularVelocityVectorStruct AngularVelocity;
  };

  struct SpatialVariantStruct {
    const DeadReckoningAlgorithmEnum8 DeadReckoningAlgorithm = DeadReckoningAlgorithmEnum8::DRM_RVW;

    SpatialRVStruct SpatialRVW;
  };

  struct PhysicalEntity {
    short int RadarCrossSectionSignatureIndex = -1;

    SpatialVariantStruct Spatial;
  };
}
