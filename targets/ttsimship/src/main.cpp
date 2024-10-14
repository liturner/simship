#include <thread>
#include <TT/simulation.h>

#include "TT/model_radar.h"
#include "TT/model_flight_dynamics.h"

#include <JSBSim/initialization/FGInitialCondition.h>

int main(int argc, char* argv[]) {
    tt::Simulation simulation;

    tt::simship::OwnshipChannel ownshipChannel;
    tt::simship::EnvironmentChannel environmentChannel;

    tt::simship::AircraftModel flightDynamics(ownshipChannel);
    tt::simship::RadarModel shipRadar(ownshipChannel, environmentChannel);

    simulation.addModel(flightDynamics);
    simulation.addModel(shipRadar);
    simulation.setTargetState(tt::Simulation::Running);
    std::thread mainThread(&tt::Simulation::main, simulation);

    // TODO: Write command input code here.

    mainThread.join();
    return 0;
}
