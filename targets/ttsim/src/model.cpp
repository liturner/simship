#include "TT/model.h"

#include "TT/logging.h"

namespace tt {
    Model::Model(const std::string_view& name, uint32_t targetFrameInterval) :
        name(name),
        targetFrameInterval(
            (targetFrameInterval)) {
    }

    bool Model::load() {
        return true;
    }

    bool Model::init() {
        return true;
    }

    bool Model::reinit() {
        return true;
    }

    bool Model::run() {
        return true;
    }

    bool Model::hold() {
        return true;
    }

    bool Model::unload() {
        return true;
    }

    const std::string_view& Model::getName() const {
        return name;
    }

    uint32_t Model::getTargetFrameInterval() const {
        return targetFrameInterval;
    }

    DataChannel::DataChannel(const std::string_view& name) :
        name_(name) {
    }
};
