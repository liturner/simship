#pragma once
#include "TT/model.h"

#include <Eigen/Core>
#include <FGFDMExec.h>
#include <JSBSim/math/FGLocation.h>

#include "data.h"

namespace tt::simship {
  class AircraftModel final : public Model {
  public:
    explicit AircraftModel(OwnshipChannel& ownshipChannel) :
      Model("AircraftModel", 0),
      outAircraftPosition_(ownshipChannel.aircraftPosition.getWriteHandle(this)),
      outAircraftRotation_(ownshipChannel.aircraftRotation.getWriteHandle(this)) {
    }

    bool load() override {
      return fdmExec_.LoadModel(
        SGPath("/home/luke/Source/jsbsim/aircraft"),
        SGPath("/home/luke/Source/jsbsim/engine"),
        SGPath("/home/luke/Source/jsbsim/systems"),
        "f16");
    }

    bool init() override {
      auto a = fdmExec_.GetPropertyCatalog();
      for (const auto& prop : a) {
        tt::log::info(prop);
      }
      return fdmExec_.RunIC();
    }

    bool run() override {
      // TODO: Update input

      bool result = fdmExec_.Run(); // execute JSBSim

      // TODO: Update output

      const JSBSim::FGLocation pos(fdmExec_.GetPropertyValue("position/long-gc-deg"),
                                   fdmExec_.GetPropertyValue("position/lat-gc-deg"),
                                   fdmExec_.GetPropertyValue("position/radius-to-vehicle-ft"));
      JSBSim::FGColumnVector3 ecef = pos;
      outAircraftPosition_->x() = JSBSim::FGFDMExec::FeetToMeters(ecef.Entry(1));
      outAircraftPosition_->y() = JSBSim::FGFDMExec::FeetToMeters(ecef.Entry(2));
      outAircraftPosition_->z() = JSBSim::FGFDMExec::FeetToMeters(ecef.Entry(3));

      return result;
    }

  private:
    JSBSim::FGFDMExec fdmExec_;

    const std::shared_ptr<Eigen::Vector3d> outAircraftPosition_;

    const std::shared_ptr<Eigen::Vector3d> outAircraftRotation_;
  };
}

