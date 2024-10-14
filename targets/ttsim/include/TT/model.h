#pragma once
#include <memory>
#include <string_view>

#include "logging.h"

namespace tt {
    class Model {
    public:
        Model(const std::string_view& name, uint32_t targetFrameInterval);

        virtual ~Model() = default;

        virtual bool load();

        virtual bool init();

        virtual bool reinit();

        virtual bool run();

        virtual bool hold();

        virtual bool unload();

        [[nodiscard]] const std::string_view& getName() const;

        [[nodiscard]] uint32_t getTargetFrameInterval() const;

    private:
        /// human readable name for the model
        const std::string_view name;

        /// minimum milliseconds between frames
        uint32_t targetFrameInterval;
    };

    template <typename T>
    class BusData {
    public:
        BusData(const T& initialValue, const std::string_view name) :
            data_(std::make_shared<T>(initialValue)),
            name_(name) {
        };

        std::shared_ptr<const T> getReadHandle(const Model* const model = nullptr) const {
            std::string message = "Read Handle: ";
            message += model == nullptr ? "Anonymous" : std::string(model->getName());
            message += " << ";
            message += std::string(name_);
            log::info(message);
            return data_;
        }

        std::shared_ptr<T> getWriteHandle(const Model* const model = nullptr) {
            std::string message = "Write Handle: ";
            message += model == nullptr ? "Anonymous" : std::string(model->getName());
            message += " >> ";
            message += std::string(name_);
            log::info(message);
            return data_;
        };

    private:
        std::shared_ptr<T> data_;

        std::string_view name_;
    };

    class DataChannel {
    public:
        virtual ~DataChannel() = default;

    protected:
        explicit DataChannel(const std::string_view& name);

    private:
        const std::string_view name_;
    };
}
