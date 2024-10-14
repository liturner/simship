#pragma once
#include <vector>

#include "model.h"

namespace tt {
    class Simulation {
    public:
        enum State {
            PreLoad,
            Loaded,
            Initialised,
            Running,
            Holding,
            Unloaded
        };

    public:
        Simulation();

        /// Adds the supplied model to the list of models to execute. Note, that the order of which the models are added
        /// is preserved during execution. It makes e.g. more sense to add control input models first so that the input
        /// is updated before the flight dynamics gets the current control values.
        ///
        /// \param model a Model to add to the simulation execution
        void addModel(Model& model);

        State getCurrentState();

        State getTargetState();

        bool setTargetState(State targetState);

        void step();

        void main();

    private:
        bool load();

        bool init();

        bool run();

        bool hold();

        bool unload();

        struct SimModel {
            Model& model;

            State currentState;
        };

        State currentState;

        State targetState_;

        std::vector<SimModel> models;
    };
}
