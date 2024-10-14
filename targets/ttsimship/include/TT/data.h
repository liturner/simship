#pragma once

#include <list>
#include <queue>
#include <Eigen/Core>

#include "TT/rpr_fom.h"

struct Echo {
    double range = 0; // meter
    double horizontalAngle = 0; // radian
    double verticalAngle = 0; // radian
    double returnPower = 0; // watt
    double radialVelocity = 0; // meters per second along the line of sight of the radar to the object
};

namespace tt::simship {
    /// A Common Synthetic Environment Channel holding information about our simulated Aircraft.
    class OwnshipChannel final : public DataChannel {
    public:
        BusData<Eigen::Vector3d> aircraftPosition;

        BusData<Eigen::Vector3d> aircraftRotation;

        BusData<Eigen::Vector3d> aircraftVelocity;

        BusData<Eigen::Vector3d> radarOffset;

        BusData<Eigen::Vector3d> radarRotation;

    public:
        OwnshipChannel() :
            DataChannel("OwnshipChannel"),
            aircraftPosition(Eigen::Vector3d(0, 0, 0), "Ownship.Position"),
            aircraftRotation(Eigen::Vector3d(0, 0, 0), "Ownship.Rotation"),
            aircraftVelocity(Eigen::Vector3d(0, 0, 0), "Ownship.Velocity"),
            radarOffset(Eigen::Vector3d(0, 0, 0), "Radar.Offset"),
            radarRotation(Eigen::Vector3d(0, 0, 0), "Radar.Rotation") {
        }
    };

    /// A Common Synthetic Environment Channel holding information about federation entities.
    class EnvironmentChannel final : public DataChannel {
    public:
        BusData<std::list<rpr_fom::PhysicalEntity>> physicalEntities;

    public:
        EnvironmentChannel() :
            DataChannel("EnvironmentChannel"),
            physicalEntities({}, "Environment.Entities") {
        }
    };
}

// Avionic Channel
namespace radarChannel {
    double horizontalFieldOfView = 50; // radian
    double verticalFieldOfView = 50; // radian
    double power = 1500; // watt
    double gain = 1; // scalar (send antenna)
    double effectiveArea = 1; // meters squared (recieve antenna)
    double minimumDetectableSignal = 9.0e-14; // watt
    std::queue<Echo> echos;
}
