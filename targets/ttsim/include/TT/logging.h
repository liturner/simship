#pragma once
#include <iostream>
#include <string_view>

namespace tt::log
{
    inline void info(const std::string_view& message)
    {
        std::cout << "INFO: " << message << std::endl;
    }
}
