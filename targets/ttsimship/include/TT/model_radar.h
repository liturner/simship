#pragma once
#include <TT/model.h>
#include <TT/transform.h>
#include <Eigen/Core>
#include "data.h"

namespace tt::simship {
  class RadarModel final : public Model {
  public:
    explicit RadarModel(const OwnshipChannel& ownshipChannel, const EnvironmentChannel& environmentChannel) :
      Model("Radar", 100),
      inAircraftPosition(ownshipChannel.aircraftPosition.getReadHandle(this)),
      inAircraftRotation(ownshipChannel.aircraftRotation.getReadHandle(this)),
      inAircraftVelocity(ownshipChannel.aircraftVelocity.getReadHandle(this)),
      inRadarOffset(ownshipChannel.radarOffset.getReadHandle(this)),
      inRadarRotation(ownshipChannel.radarRotation.getReadHandle(this)),
      inEnvironmentEntities(environmentChannel.physicalEntities.getReadHandle(this)),
      inHorizontalFieldOfView(&radarChannel::horizontalFieldOfView),
      inVerticalFieldOfView(&radarChannel::verticalFieldOfView),
      inPower(&radarChannel::power),
      inGain(&radarChannel::gain),
      inEffectiveArea(&radarChannel::effectiveArea),
      inMinimumDetectableSignal(&radarChannel::minimumDetectableSignal),
      outEchos(&radarChannel::echos) {
    }

    bool load() override {
      ownshipXform = tt::Transform(*inAircraftPosition, *inAircraftRotation);
      radarXform = tt::Transform(*inRadarOffset, *inRadarRotation);
      radarXform.setParent(&ownshipXform);
      return true;
    }

    bool run() override {
      // Update xforms
      ownshipXform.setLocalTranslation(*inAircraftPosition);
      ownshipXform.setLocalRotationEuler(*inAircraftRotation);
      radarXform.setLocalTranslation(*inRadarOffset);
      radarXform.setLocalRotationEuler(*inRadarRotation);

      // Pre-compute useful values
      const double halfHorizontalFieldOfView = *inHorizontalFieldOfView * 0.5;
      const double halfVerticalFieldOfView = *inVerticalFieldOfView * 0.5;

      for (auto entity = inEnvironmentEntities->begin(); entity != inEnvironmentEntities->end(); ++entity) {
        const tt::Transform entityWorldXform(
          entity->Spatial.SpatialRVW.WorldLocation.X,
          entity->Spatial.SpatialRVW.WorldLocation.Y,
          entity->Spatial.SpatialRVW.WorldLocation.Z,
          entity->Spatial.SpatialRVW.Orientation.Phi,
          entity->Spatial.SpatialRVW.Orientation.Theta,
          entity->Spatial.SpatialRVW.Orientation.Psi);
        const tt::Transform entityInRadarSpace(radarXform.toLocalTransform(entityWorldXform));
        const auto otherOffset = entityInRadarSpace.getLocalTranslation();

        // Is the entity behind us?
        if (otherOffset.x() < 0) {
          continue;
        }

        // Is the entity within our beam?
        const double horizontalAngle = std::atan2(otherOffset.y(), otherOffset.x());
        const double verticalAngle = std::atan2(otherOffset.z(), otherOffset.x());
        if (abs(horizontalAngle) > halfHorizontalFieldOfView || abs(verticalAngle) > halfVerticalFieldOfView) {
          continue;
        }

        // Radar Cross Section Check
        const double radarCrossSection = 3.5;
        const double distance = otherOffset.norm();
        // const double radarCrossSection = getRadarCrossSection(entity);
        // TODO: Apply weather reduction to the radar power
        // TODO: Consider a function which can produce a more accurate Radar Cross Section
        const double returnPower = ((*inPower * *inGain) / (M_PI_4 * (distance * distance)))
          * radarCrossSection
          * (1.0 / (M_PI_4 * (distance * distance)))
          * *inEffectiveArea;

        if (returnPower < *inMinimumDetectableSignal) {
          continue;
        }

        // Calculate radial velocity
        const Eigen::Vector3d entityWorldVelocity(entity->Spatial.SpatialRVW.VelocityVector.XVelocity,
                                                  entity->Spatial.SpatialRVW.VelocityVector.YVelocity,
                                                  entity->Spatial.SpatialRVW.VelocityVector.ZVelocity);
        const Eigen::Vector3d entityVelocityInRadarSpace(radarXform.toLocalVector(entityWorldVelocity));
        const Eigen::Vector3d radarVelocityInRadarSpace(radarXform.toLocalVector(*inAircraftVelocity));
        const Eigen::Vector3d entityVelocityRelativeToRadar(entityVelocityInRadarSpace - radarVelocityInRadarSpace);

        // If we are here, the radar would have a good chance of an Echo
        Echo radarEcho;
        radarEcho.range = distance;
        radarEcho.horizontalAngle = horizontalAngle;
        radarEcho.verticalAngle = verticalAngle;
        radarEcho.radialVelocity = entityVelocityRelativeToRadar.x();
        radarEcho.returnPower = returnPower;

        outEchos->push(radarEcho);
      }

      return true;
    }

  private:
    tt::Transform ownshipXform;

    tt::Transform radarXform;

  private:
    const std::shared_ptr<const Eigen::Vector3d> inAircraftPosition;

    const std::shared_ptr<const Eigen::Vector3d> inAircraftRotation;

    const std::shared_ptr<const Eigen::Vector3d> inAircraftVelocity;

    const std::shared_ptr<const Eigen::Vector3d> inRadarOffset;

    const std::shared_ptr<const Eigen::Vector3d> inRadarRotation;

    const std::shared_ptr<const std::list<rpr_fom::PhysicalEntity>> inEnvironmentEntities;

    const double* inHorizontalFieldOfView;

    const double* inVerticalFieldOfView;

    const double* inPower;

    const double* inGain;

    const double* inEffectiveArea;

    const double* inMinimumDetectableSignal;

    std::queue<Echo>* outEchos;
  };
}
