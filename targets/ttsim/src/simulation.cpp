#include "TT/simulation.h"

#include "TT/logging.h"

tt::Simulation::Simulation() :
    currentState(PreLoad),
    targetState_(PreLoad) {
}

void tt::Simulation::addModel(Model& model) {
    SimModel simModel{model, PreLoad};
    models.emplace_back(simModel);
}

bool tt::Simulation::setTargetState(const State targetState) {
    // TODO: Return false if illegal transition

    targetState_ = targetState;
    return true;
}

void tt::Simulation::step() {
    // The most common case here for efficiency
    if (targetState_ == Running && (currentState == Running || currentState == Initialised)) {
        if (run()) {
            currentState = Running;
        }
        return;
    }

    if (targetState_ >= Loaded && currentState < Loaded) {
        if (load()) {
            currentState = Loaded;
        }
        return;
    }

    if (targetState_ >= Initialised && currentState < Initialised) {
        if (init()) {
            currentState = Initialised;
        }
        return;
    }
}

void tt::Simulation::main() {
    log::info("main()");

    while (currentState != Unloaded) {
        step();
    }
}

bool tt::Simulation::load() {
    log::info("load()");
    bool allLoaded = true;
    for (auto& model : models) {
        if (model.currentState < Loaded) {
            if (model.model.load()) {
                model.currentState = Loaded;
            }
            else {
                allLoaded = false;
            }
        }
    }
    return allLoaded;
}

bool tt::Simulation::init() {
    log::info("init()");
    for (auto& model : models) {
        if (model.model.init() == false) {
            return false;
        }
    }
    return true;
}

bool tt::Simulation::run() {
    for (auto& model : models) {
        if (model.model.run() == false) {
            return false;
        }
    }
    return true;
}

bool tt::Simulation::hold() {
    for (auto& model : models) {
        if (model.model.hold() == false) {
            return false;
        }
    }
    return true;
}

bool tt::Simulation::unload() {
    for (auto& model : models) {
        if (model.model.unload() == false) {
            return false;
        }
    }
    return true;
}
